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
void motor_run_position();
int pid(int currentPosition, int wantedPosition);
