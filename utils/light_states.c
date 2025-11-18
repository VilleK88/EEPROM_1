#include "light_states.h"

/*void run_light_sm(light_sm *smi) {
    switch (smi->state) {
        case lights_on:
            // do something
            break;
        case lights_off:
            // do something
            break;
        default:
            break;
    }
}*/

bool check_led_states() {
    for (int i = 251; i < 256; i+=2) {
        led_state ls;
        ls.state = read_byte(i);
        ls.not_state = read_byte(i+1);
        if (!led_state_is_valid(&ls))
            return false;
    }
    return true;
}

void set_led_state(led_state *ls, uint8_t const value) {
    ls->state = value;
    ls->not_state = ~value;
}

bool led_state_is_valid(led_state *ls) {
    return ls->state == (uint8_t) ~ls->not_state;
}

void init_led_states() {
    light_switch(LED_L, 251, LIGHT_OFF);
    light_switch(LED_M, 253, LIGHT_ON);
    light_switch(LED_R, 255, LIGHT_OFF);
}