#pragma once

#include <sam.h>

#include <stdint.h>
#include <stdio.h>



// signal period should be 20ms
// pulse-width of 1.5ms 
// max angle 0.9ms and 2.1ms


void servo_init(void);
void servo_enable(void);
void servo_disable(void);
void servo_toggle(void);
void servo_set_pulse_width(uint16_t pwm);