#pragma once

#include <stdio.h>
#include <stdint.h>

#include "encoder.h"


void motor_init(void);
void motor_set_direction(uint8_t direction);
void motor_set_speed(uint16_t speed);
void motor_set_position(int position);
void motor_increase_position(uint8_t increment);
void motor_decrease_position(uint8_t increment);
void motor_run_position(void);  // called at a fixed rate from the TC0 interrupt, not from main
void motor_print_status(void);
uint32_t motor_get_ticks(void);  // control ticks since start, motor_tick_hz per second
#define motor_tick_hz 200        // must match control_hz in motor.c
