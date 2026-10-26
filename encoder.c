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

    PMC->PMC_PCER1 = (1u << (ID_TC6 - 32));

    PIOC->PIO_ABSR |= (1u << 25) | (1u << 26);

    PIOC->PIO_PDR = (1u << 25) | (1u << 26);

    TC2->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_XC0 | TC_CMR_ABETRG | TC_CMR_ETRGEDG_RISING;

    TC2->TC_BMR = TC_BMR_QDEN | TC_BMR_POSEN;

    TC2->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;

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