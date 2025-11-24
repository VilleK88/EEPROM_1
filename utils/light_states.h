#ifndef LIGHT_STATES_H
#define LIGHT_STATES_H

#include "../main.h"

#include "hardware/pwm.h"

#define LIGHT_ON 1
#define LIGHT_OFF 0

typedef enum {
    lights_on,
    lights_off
} light_st;

typedef struct light_sm {
    light_st state;
} light_sm;

typedef struct led_state {
    uint8_t state;
    uint8_t not_state;
} led_state;

bool check_if_led_states_are_valid();
bool light_on(uint16_t addr);
void set_led_state(led_state *ls, uint8_t value);
bool led_state_is_valid(led_state *ls);
void init_led_states(bool valid);
void init_led_state(uint led, uint16_t addr, uint8_t value);
void light_switch(uint led, uint16_t addr); // Turn lights on/off
void set_brightness(uint led, uint brightness); // Increase/decrease lighting
void print_led_states();
void print_led_state(char *led, uint16_t addr);

#endif