#include "light_states.h"

bool check_if_led_states_are_valid() {
    for (int i = 251; i < 256; i+=2) {
        led_state ls;
        ls.state = read_byte(i);
        ls.not_state = read_byte(i+1);
        if (!led_state_is_valid(&ls))
            return false;
    }
    return true;
}

bool light_on(const uint16_t addr) {
    if (read_byte(addr) == 1)
        return true;
    return false;
}

void set_led_state(led_state *ls, uint8_t const value) {
    ls->state = value;
    ls->not_state = ~value;
}

bool led_state_is_valid(led_state *ls) {
    return ls->state == (uint8_t) ~ls->not_state;
}

void init_led_states(const bool valid) {
    if (!valid) {
        init_led_state(LED_L, 251, LIGHT_OFF);
        init_led_state(LED_M, 253, LIGHT_ON);
        init_led_state(LED_R, 255, LIGHT_OFF);
    }
    else {
        for (int i = 0; i < LEDS_SIZE; i++) {
            if (light_on(leds_addr[i])) {
                set_brightness(leds[i], BR_MID);
            }
        }
    }
}

void init_led_state(uint led, uint16_t addr, uint8_t value) {
    led_state ls;
    set_led_state(&ls, value);
    write_byte(addr, ls.state);
    write_byte(addr+1, ls.not_state);
    if (value == 1) {
        set_brightness(led, BR_MID);
    }
    else if (value == 0) {
        set_brightness(led, 0);
    }
}

void light_switch(const uint led, const uint16_t addr) {
    led_state ls;
    if (!light_on(addr)) {
        set_led_state(&ls, 1);
        write_byte(addr, ls.state);
        write_byte(addr+1, ls.not_state);
        set_brightness(led, BR_MID);
    }
    else {
        set_led_state(&ls, 0);
        write_byte(addr, ls.state);
        write_byte(addr+1, ls.not_state);
        set_brightness(led, 0);
    }
    print_led_states();
}

void set_brightness(const uint led, const uint brightness) {
    const uint slice = pwm_gpio_to_slice_num(led);  // Get PWM slice for LED pin
    const uint chan  = pwm_gpio_to_channel(led); // Get PWM channel (A/B)
    pwm_set_chan_level(slice, chan, brightness); // Update duty cycle value
}

void print_led_states() {
    char *char_leds[] = {"Middle", "Left", "Right"};
    for (int i = 0; i < LEDS_SIZE; i++) {
        print_led_state(char_leds[i], leds_addr[i]);
        if (i < LEDS_SIZE-1)
            printf(", ");
    }
    printf(".\r\n");
    printf("The number of seconds since power-on: %.2f.\r\n", (float)time_us_64() / 1000000);
}

void print_led_state(char *led, const uint16_t addr) {
    printf("%s led state: ", led);
    if (light_on(addr))
        printf("On");
    else
        printf("Off");
}