#include <stdio.h>
#include <stdbool.h>

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/util/queue.h"

#define CLK_DIV 125 // PWM clock divider
#define TOP 999 // PWM counter top value

#define SW_R 7 // right button - decreases brightness
#define SW_M 8 // middle button - light switch
#define SW_L 9 // left button - increases brightness
#define BUTTONS_SIZE 3 // how many buttons

#define LED_R 22 // right LED
#define LED_M 21 // middle LED
#define LED_L 20 // left LED
#define LEDS_SIZE 3 // how many LEDs

#define BR_RATE 4 // step size for brightness changes
#define MAX_BR (TOP + 1) // max brightness
#define BR_MID (MAX_BR / 2) // 50% brightness level

#define DEBOUNCE_MS 20 // Debounce delay in milliseconds

#define BAUD_RATE 100000

#define I2C i2c0
// I2C pins
#define I2C_SDA 16 // Serial Data Line
#define I2C_SCL 17 // Serial Clock Line
#define I2C_SIZE 2

#define EEPROM_ADDRESS 0x50 // EEPROM I2C address

static const uint i2cs[] = {I2C_SDA, I2C_SCL};

typedef struct led_state {
    uint8_t state;
    uint8_t not_state;
} led_state;

// Type of event coming from the interrupt callback
typedef enum { EV_SW_M, EV_SW_L, EV_SW_R } event_type;

// Generic event passed from ISR to main loop through a queue
typedef struct {
    event_type type; // EVENT_BUTTON
    int32_t data; // BUTTON: 1 = press, 0 = release;
} event_t;

// Global event queue used by ISR (Interrupt Service Routine) and main loop
static queue_t events;

void gpio_callback(uint gpio, uint32_t event_mask);
void init_buttons(const uint *buttons); // Initialize buttons
void init_leds(const uint *leds); // Initialize LED pins
uint init_i2c();
bool light_switch(const uint *leds, uint brightness, bool on); // Turn lights on/off
void set_brightness(const uint *leds, uint brightness); // Increase/decrease lighting
uint clamp(int br); // returns value between 0 and TOP
void set_led_state(led_state *ls, uint8_t value);
bool led_state_is_valid(led_state *ls);
void write_byte(uint16_t address, uint8_t value);
uint8_t read_byte(uint16_t address);

int main() {
    const uint buttons[] = {SW_R, SW_M, SW_L};
    const uint leds[] = {LED_R, LED_M, LED_L};
    uint brightness = BR_MID; // LEDs brightness value

    // Initialize chosen serial port
    stdio_init_all();
    // Initialize buttons
    init_buttons(buttons);
    // Initialize LED pins
    init_leds(leds);

    // Initialize I2C
    uint baud = init_i2c();
    printf("I2C ready (actual baud %u)\r\n", baud);

    write_byte(0, 0xA5);
    write_byte(1, 0xBC);

    uint8_t value_0 = read_byte(0);
    uint8_t value_1 = read_byte(1);

    printf("Read from EEPROM:\r\n");
    printf("Address 0: 0x%02X\r\n", value_0);
    printf("Address 1: 0x%02X\r\n", value_1);

    bool lightsOn = false;
    bool SW_L_down = false;
    bool SW_R_down = false;

    event_t event;
    while (true) {
        // Process pending events from the queue
        while (queue_try_remove(&events, &event)) {
            // React only to button press (falling edge event, data == 1)
            if (event.type == EV_SW_M && event.data == 1) {
                if (!lightsOn) {
                    lightsOn = light_switch(leds, brightness, true);
                }
                else {
                    // If LEDs are on and brightness is 0%, restore to 50%
                    if (brightness <= 0) {
                        brightness = BR_MID;
                        set_brightness(leds, BR_MID);
                    }
                    // Otherwise turn lights off
                    else {
                        lightsOn = light_switch(leds, 0, false);
                    }
                }
            }

            if (lightsOn) {
                if (event.type == EV_SW_R)
                    SW_R_down = event.data == 1;
                if (event.type == EV_SW_L)
                    SW_L_down = event.data == 1;
            }
        }

        if (lightsOn) {
            // Increase lighting
            if (SW_R_down) {
                brightness = clamp((int)brightness + BR_RATE);
                set_brightness(leds, brightness);
            }
            // Decrease lighting
            if (SW_L_down) {
                brightness = clamp((int)brightness - BR_RATE);
                set_brightness(leds, brightness);
            }
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
            printf("SW_M pressed\r\n");
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
            printf("SW_L pressed\r\n");
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
            printf("SW_R pressed\r\n");
        }
    }
}

void init_buttons(const uint *buttons) {
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

void init_leds(const uint *leds) {
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

uint init_i2c() {
    const uint baud = i2c_init(I2C, BAUD_RATE);
    for (int i = 0; i < I2C_SIZE; i++) {
        gpio_set_function(i2cs[i], GPIO_FUNC_I2C);
        gpio_pull_up(i2cs[i]);
    }
    return baud;
}

bool light_switch(const uint *leds, const uint brightness, const bool on) {
    if (on) {
        set_brightness(leds, brightness);
        return true;
    }
    set_brightness(leds, 0);
    return false;
}

void set_brightness(const uint *leds, const uint brightness) {
    // Set PWM duty cycle for each LED to match the desired brightness
    for (int i = 0; i < LEDS_SIZE; i++) {
        const uint slice = pwm_gpio_to_slice_num(leds[i]);  // Get PWM slice for LED pin
        const uint chan  = pwm_gpio_to_channel(leds[i]); // Get PWM channel (A/B)
        pwm_set_chan_level(slice, chan, brightness); // Update duty cycle value
    }
}

uint clamp(const int br) {
    // Limit brightness value to valid PWM range [0, MAX_BR]
    if (br < 0) return 0; // Lower bound
    if (br > MAX_BR) return MAX_BR; // Upper bound
    return br; // Within range
}

void set_led_state(led_state *ls, uint8_t const value) {
    ls->state = value;
    ls->not_state = ~value;
}

bool led_state_is_valid(led_state *ls) {
    return ls->state == (uint8_t) ~ls->not_state;
}

void write_byte(uint16_t const address, uint8_t const value) {
    uint8_t buffer[3];

    // MSB (Most Significant Byte) address
    buffer[0] = address >> 8 & 0xFF; // 0xFF
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