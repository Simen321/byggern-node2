#include "solednoid.h"


uint8_t solenoid_state = 1;

void solenoid_init(void){
    PIOC->PIO_PER = (1u << 16); // used as gpio not peripheral
    PIOC->PIO_OER = (1u << 16); //config as output
}
void solenoid_toggle(void){
    solenoid_on();
    int i = 0;
    while (i < 25000) {
        i++;
    }
    solenoid_off();
    /*
    if (solenoid_state) {
    printf("off\r\n");
        
        solenoid_off();
    } else {
    printf("on\r\n");

        solenoid_on();
    }*/
}

void solenoid_on(void){
    solenoid_state = 1;
    PIOC->PIO_SODR = (1u << 16);
}
void solenoid_off(void){
    solenoid_state = 0;
    PIOC->PIO_CODR = (1u << 16);
}