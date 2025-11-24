#ifndef LIGHT_STATES_H
#define LIGHT_STATES_H

#include "../main.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/util/queue.h"

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

void run_light_sm(light_sm *smi);
bool check_if_led_states_are_valid();
bool light_on(uint16_t addr);
void set_led_state(led_state *ls, uint8_t value);
bool led_state_is_valid(led_state *ls);
void init_led_states(bool valid);
void init_led_state(uint led, uint16_t addr, uint8_t value);

#endif