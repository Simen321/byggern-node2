#include "encoder.h"

const int8_t transitionTable[16] = 
{
    0, -1, +1, 0,
    +1, 0, 0, -1,
    -1, 0, 0, +1,
    0, +1, -1, 0
};



volatile int32_t encoderCount = 0;
volatile uint8_t previousState = 0;




void encoder_init(void) {

    // TC2 channel 0 (ID_TC6) in quadrature decoder mode, the position is counted in TC_CV.
    // channel A on arduino 5, in sam: PC25 (TIOA6). channel B on arduino 4, in sam: PC26 (TIOB6)

    PIOC->PIO_ABSR |= (1u << 25) | (1u << 26); // set PC25 and PC26 to peripheral type B.
    PIOC->PIO_PDR = (1u << 25) | (1u << 26); // disable GPIO on pins.

    PMC->PMC_PCER1 = (1u << (ID_TC6 - 32)); // turn the TC2 timer on
    TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_XC0 | TC_CMR_ABETRG | TC_CMR_ETRGEDG_RISING; // count pulses from decoder
    TC2->TC_BMR = TC_BMR_QDEN | TC_BMR_POSEN; // quadrature decoder on, position mode
    TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}


// make the current position 0 (used after homing)
void encoder_reset(void) {
    TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_SWTRG; // software trigger resets the counter
}


int16_t encoder_read(void) {
    return (int16_t)TC2->TC_CHANNEL[0].TC_CV;
}


/*
void updateEncoder() {

    uint8_t chnA = 
    uint8_t chaB = 

    uin8_t currentState = (a << 1) | b;
    uint8_t index = (previousState << 2) | currentState;

    encoderCount += transitionTable[index];
    previousState = currentState;
}*/