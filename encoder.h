#pragma once

#include <sam.h>


// chaA and ChB are connected to D4 and D5. PC26 PC25

// Use PWM4, which is connected to BOTH PC26 and PC29
// PC25/26 are used by channel 0 of TC2
// Set XC0 as the selected clock for channel


// pan:
// 1. Sync up to read often enough.
// 2. Compare with previous signal:
        // if rising_edge:
                // increment
        // if falling_edge:
                //decrement

// 3. Store signal if not done



void encoder_init(void);
int16_t encoder_read(void);
//void encoder_sample(void);
