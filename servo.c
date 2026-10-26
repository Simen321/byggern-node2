#include "servo.h"

void servo_init(void){




    // we first enable the Power management controller PMC, the PWM is id 36.
    // arduino port 21 is port b bit 13
    PMC->PMC_PCER0 = (1u << 12);
    PMC->PMC_PCER1 = (1u << 4); // instead of PMC->PMC_PCER1 |= (1u << 4) because its a write only register

    PWM->PWM_DIS = (1u << 1);

    PIOB->PIO_ABSR |= (1u << 13); // Turns on the bit on bit 13 which turns on peripheral B.
    PIOB->PIO_PDR = (1u << 13);  // PDR: PIO disable register, disables the pins GPIO function and lets it be controller by the peripheral controller instead

    PWM->PWM_CH_NUM[1].PWM_CMR = PWM_CMR_CPRE_CLKA | PWM_CMR_CPOL; //Sets the clock prescaler (CPRE) to clockA and CPOL 1, which means waveform starts high.

    PWM->PWM_CLK = PWM_CLK_DIVA(84) | PWM_CLK_PREA(0); // sets DIVA to 84, which makes the frequency on clockA be 84Mhz/84*1 = 1Mhz -> 1us for an increment

    PWM->PWM_CH_NUM[1].PWM_CPRD = 20000u; //20ms set on the channle period register
    PWM->PWM_CH_NUM[1].PWM_CDTY = 1500u; // 1.5ms set on the channel duty cycle register. basically idle value between 900-2100
}

uint8_t servo_on = 0;

void servo_enable(){
    servo_on = 1;
    PWM->PWM_ENA = (1u << 1);
}
void servo_disable() {
    servo_on = 0;
    PWM->PWM_DIS = (1u << 1);
}
void servo_toggle() {
    if (servo_on) {
        servo_disable();
    } else {
        servo_enable();
    }
}


void servo_set_pulse_width(uint16_t pwm) {
    pwm = min(pwm, 2100);
    pwm = max(pwm, 900);
    //printf("actual: %d\r\n", pwm);
    PWM->PWM_CH_NUM[1].PWM_CDTY = pwm;
}

