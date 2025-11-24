#include "main.h"

int main() {
    // Initialize chosen serial port
    stdio_init_all();
    // Initialize buttons
    init_buttons();
    // Initialize LED pins
    init_leds();
    // Initialize I2C
    init_i2c();

    if (check_if_led_states_are_valid()) {
        printf("States are correct.\r\n");
        init_led_states(true);
    }
    else {
        printf("States not correct.\r\n");
        init_led_states(false);
    }
    print_led_states();

    event_t event;
    while (true) {
        // Process pending events from the queue
        while (queue_try_remove(&events, &event)) {

            if (event.type == EV_SW_R && event.data == 1)
                light_switch(LED_R, LED_R_ADDR);

            if (event.type == EV_SW_M && event.data == 1)
                light_switch(LED_M, LED_M_ADDR);

            if (event.type == EV_SW_L && event.data == 1)
                light_switch(LED_L, LED_L_ADDR);
        }

        sleep_ms(10); // 10 ms delay (0.01 second) to reduce CPU usage
    }
}

// Interrupt callback for pressing buttons
void gpio_callback(uint const gpio, uint32_t const event_mask) {
    const uint32_t now = to_ms_since_boot(get_absolute_time());
    // Button press/release with debounce to ensure one physical press counts as one event
    if (gpio == SW_M) {
        static uint32_t last_ms_m = 0; // Store last interrupt time
        // Detect button release (rising edge)
        if (event_mask & GPIO_IRQ_EDGE_RISE && now - last_ms_m >= DEBOUNCE_MS) {
            last_ms_m = now;
            const event_t event = { .type = EV_SW_M, .data = 0 };
            queue_try_add(&events, &event); // Add event to queue
        }

        // Detect button press (falling edge)
        if (event_mask & GPIO_IRQ_EDGE_FALL && now - last_ms_m >= DEBOUNCE_MS){
            last_ms_m = now;
            const event_t event = { .type = EV_SW_M, .data = 1 };
            queue_try_add(&events, &event); // Add event to queue
        }
    }

    if (gpio == SW_L) {
        static uint32_t last_ms_l = 0; // Store last interrupt time

        // Detect button release (rising edge)
        if (event_mask & GPIO_IRQ_EDGE_RISE && now - last_ms_l >= DEBOUNCE_MS) {
            last_ms_l = now;
            const event_t event = { .type = EV_SW_L, .data = 0 };
            queue_try_add(&events, &event); // Add event to queue
        }

        // Detect button press (falling edge)
        if (event_mask & GPIO_IRQ_EDGE_FALL && now - last_ms_l >= DEBOUNCE_MS) {
            last_ms_l = now;
            const event_t event = { .type = EV_SW_L, .data = 1 };
            queue_try_add(&events, &event); // Add event to queue
        }
    }

    if (gpio == SW_R) {
        static uint32_t last_ms_r = 0; // Store last interrupt time

        if (event_mask & GPIO_IRQ_EDGE_RISE && now - last_ms_r >= DEBOUNCE_MS) {
            last_ms_r = now;
            const event_t event = { .type = EV_SW_R, .data = 0 };
            queue_try_add(&events, &event);
        }
        if (event_mask & GPIO_IRQ_EDGE_FALL && now - last_ms_r >= DEBOUNCE_MS) {
            last_ms_r = now;
            const event_t event = { .type = EV_SW_R, .data = 1 };
            queue_try_add(&events, &event);
        }
    }
}

void init_buttons() {
    // Initialize event queue for Interrupt Service Routine (ISR)
    // 32 chosen as a safe buffer size: large enough to handle bursts of interrupts
    // without losing events, yet small enough to keep RAM usage minimal.
    queue_init(&events, sizeof(event_t), 32);

    for (int i = 0; i < BUTTONS_SIZE; i++) {
        gpio_init(buttons[i]); // Initialize GPIO pin
        gpio_set_dir(buttons[i], GPIO_IN); // Set as input
        gpio_pull_up(buttons[i]); // Enable internal pull-up resistor (button reads high = true when not pressed)
        // Configure button interrupt and callback
        gpio_set_irq_enabled_with_callback(buttons[i], GPIO_IRQ_EDGE_FALL |
            GPIO_IRQ_EDGE_RISE, true, &gpio_callback);
    }
}

void init_leds() {
    // Track which PWM slices (0-7) have been initialized
    bool slice_ini[8] = {false};

    // Get default PWM configuration
    pwm_config config = pwm_get_default_config();
    // Set clock divider
    pwm_config_set_clkdiv_int(&config, CLK_DIV);
    // Set wrap (TOP)
    pwm_config_set_wrap(&config, TOP);

    for (int i = 0; i < LEDS_SIZE; i++) {
        // Get slice and channel for your GPIO pin
        const uint slice = pwm_gpio_to_slice_num(leds[i]);
        const uint chan = pwm_gpio_to_channel(leds[i]);

        // Disable PWM while configuring
        pwm_set_enabled(slice, false);

        // Initialize each slice once (sets divider and TOP for both A/B)
        if (!slice_ini[slice]) {
            pwm_init(slice, &config, false); // Do not start yet
            slice_ini[slice] = true;
        }

        // Set compare value (CC) to define duty cycle
        pwm_set_chan_level(slice, chan, 0);
        // Select PWM model for your pin
        gpio_set_function(leds[i], GPIO_FUNC_PWM);
        // Start PWM
        pwm_set_enabled(slice, true);
    }
}

void init_i2c() {
    i2c_init(I2C, BAUD_RATE);
    for (int i = 0; i < I2C_SIZE; i++) {
        gpio_set_function(i2cs[i], GPIO_FUNC_I2C);
        gpio_pull_up(i2cs[i]);
    }
}

uint clamp(const int br) {
    // Limit brightness value to valid PWM range [0, MAX_BR]
    if (br < 0) return 0; // Lower bound
    if (br > MAX_BR) return MAX_BR; // Upper bound
    return br; // Within range
}

void write_byte(uint16_t const address, uint8_t const value) {
    uint8_t buffer[3];

    // MSB (Most Significant Byte) address
    buffer[0] = address >> 8 & 0xFF;
    // LSB (Least Significant Byte) address
    buffer[1] = address & 0xFF;
    // Actual data, one byte
    buffer[2] = value;

    // Send 3 bytes:
    // - 2 bytes of address
    // - 1 byte of data
    i2c_write_blocking(I2C, EEPROM_ADDRESS, buffer, 3, false);
    sleep_ms(5);
}

uint8_t read_byte(uint16_t const address) {
    uint8_t buffer[2];
    uint8_t data;
    buffer[0] = address >> 8 & 0xFF; // MSB
    buffer[1] = address & 0xFF; // LSB
    i2c_write_blocking(I2C, EEPROM_ADDRESS, buffer, 2,true);
    i2c_read_blocking(I2C, EEPROM_ADDRESS, &data, 1, false);
    return data;
}