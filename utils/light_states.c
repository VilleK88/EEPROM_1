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
        init_led_state(LED_M, 251, LIGHT_ON);
        for (int i = 1; i < LEDS_SIZE; i++) {
            init_led_state(leds[i], leds_addr[i], LIGHT_OFF);
        }
    }
    else {
        for (int i = 0; i < LEDS_SIZE; i++) {
            if (light_on(leds_addr[i])) {
                //light_switch(leds[i], leds_addr[i]);
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
}

void set_brightness(const uint led, const uint brightness) {
    const uint slice = pwm_gpio_to_slice_num(led);  // Get PWM slice for LED pin
    const uint chan  = pwm_gpio_to_channel(led); // Get PWM channel (A/B)
    pwm_set_chan_level(slice, chan, brightness); // Update duty cycle value
}