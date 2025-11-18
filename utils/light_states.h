#ifndef LIGHT_STATES_H
#define LIGHT_STATES_H

#include "../main.h"

#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "pico/util/queue.h"

#define LIGHT_ON 0xA0
#define LIGHT_OFF 0x5F

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
bool check_led_states();
void set_led_state(led_state *ls, uint8_t value);
bool led_state_is_valid(led_state *ls);
void init_led_states();

#endif