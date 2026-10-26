#include "diode.h"

void diode_init(void){
    PMC->PMC_PCER1 = (1u << (ID_ADC - 32));
    
    ADC->ADC_CHER = (1u << 7); // channel 7 is A0
}

uint16_t diode_read(void){
    ADC->ADC_CR = ADC_CR_START;

    while (!(ADC->ADC_ISR & ADC_ISR_EOC7));

    return ADC->ADC_CDR[7] & 0x0FFF;

}