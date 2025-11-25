#ifndef LIGHT_STATES_H
#define LIGHT_STATES_H

#include "../main.h"

#include "hardware/pwm.h"

#define LIGHT_ON 1
#define LIGHT_OFF 0

#define LED_L 20 // left LED
#define LED_M 21 // middle LED
#define LED_R 22 // right LED
#define LEDS_SIZE 3 // how many LEDs
static const uint leds[] = {LED_L, LED_M, LED_R};

#define LED_L_ADDR 32762
#define LED_M_ADDR 32764
#define LED_R_ADDR 32766
static const uint leds_addr[] = {LED_L_ADDR, LED_M_ADDR, LED_R_ADDR};

#define BR_RATE 4 // step size for brightness changes
#define MAX_BR (TOP + 1) // max brightness
#define BR_MID (MAX_BR / 2) // 50% brightness level

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