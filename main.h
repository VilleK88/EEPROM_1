#ifndef MAIN_H
#define MAIN_H

#include <stdio.h>
#include <stdbool.h>
#include <string.h>

#include "utils/light_states.h"

#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "pico/util/queue.h"

#define CLK_DIV 125 // PWM clock divider
#define TOP 999 // PWM counter top value

#define SW_R 7 // right button - decreases brightness
#define SW_M 8 // middle button - light switch
#define SW_L 9 // left button - increases brightness
#define BUTTONS_SIZE 3 // how many buttons
static const uint buttons[] = {SW_R, SW_M, SW_L};

#define LED_L 20 // left LED
#define LED_M 21 // middle LED
#define LED_R 22 // right LED
#define LEDS_SIZE 3 // how many LEDs
static const uint leds[] = {LED_L, LED_M, LED_R};

#define LED_L_ADDR 551
#define LED_M_ADDR 553
#define LED_R_ADDR 555
static const uint leds_addr[] = {LED_L_ADDR, LED_M_ADDR, LED_R_ADDR};

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
void init_buttons(); // Initialize buttons
void init_leds(); // Initialize LED pins
void init_i2c();
uint clamp(int br); // returns value between 0 and TOP
void write_byte(uint16_t address, uint8_t value);
uint8_t read_byte(uint16_t address);

#endif