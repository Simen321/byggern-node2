#include "motor.h"
#include <sam.h>


#define k_p 1
#define k_i 0.1
#define k_d 0

#define maxPosition 2000
#define minPosition 0


int wantedPosition = 0;

typedef enum {
    POSITION,
    VELOCITY
} Controltype;



void motor_init(void) {

    PMC->PMC_PCER1 = (1u << (ID_PWM - 32));

    PIOB->PIO_ABSR |= (1u << 12);
    PIOB->PIO_PDR   = (1u << 12);

    PIOC->PIO_PER   = (1u << 23);
    PIOC->PIO_CODR = (1u << 23);
    PIOC->PIO_OER = (1u << 23);

    PWM->PWM_DIS = (1u << 0);

    PWM->PWM_CH_NUM[0].PWM_CMR = (1u << 9);

    PWM->PWM_CH_NUM[0].PWM_CPRD = 4200u;
    PWM->PWM_CH_NUM[0].PWM_CDTY = 0;

    PWM->PWM_ENA = (1u << 0);


}




uint8_t previousDirection = 0;

void motor_set_direction(uint8_t direction) {
    // disable if swapping direction
    if (direction != previousDirection){
        PWM->PWM_CH_NUM[0].PWM_CDTYUPD = 0;
        int i = 25000;
        while (i >= 0) {
            i--;
        }
    }

    if (direction == 0) {
        PIOC->PIO_SODR = (1u << 23);
    } else {
        PIOC->PIO_CODR = (1u << 23);
    }
}

void motor_set_speed(uint16_t speed) {
    PWM->PWM_CH_NUM[0].PWM_CDTYUPD = speed;
}



// 0 is to the left, so towards the cable out side of the box
void motor_set_position(int position) {
    if (((wantedPosition) < maxPosition) && ((wantedPosition) > minPosition)) {
        wantedPosition = position;
    }
}

void motor_increase_position(uint8_t increment) {
    if (((wantedPosition+increment) < maxPosition) && ((wantedPosition+increment) > minPosition)) {
        wantedPosition += increment;
    }
}
void motor_decrease_position(uint8_t increment) {
    if (((wantedPosition-increment) < maxPosition) && ((wantedPosition-increment) > minPosition)) {
        wantedPosition -= increment;
    }
}

void motor_run_position() {
    // calculate pid
    int currentPosition = encoder_read();
    int controlEffort = pid(currentPosition, wantedPosition);
    uint8_t direction = 0;

    // handling of mins and maxes
    uint16_t speed = min(controlEffort, 4000);
    speed = max(0, controlEffort);
    printf("Motor wanted: %d, motor current: %d, motor speed: %d", wantedPosition, currentPosition, speed);
    if (wantedPosition > 0) {
        direction = 0;
    } else {
        direction = 1;
    }


    // give control effort to motor
    motor_set_direction(direction);
    motor_set_speed(speed);
}


int pid(int currentPosition, int newPos) {
    int error = 0;
    int controlEffort = 0;
    error = currentPosition - newPos;
    
    controlEffort = (error * k_d);
    return controlEffort;
}