#include <stdio.h>
#include <stdarg.h>
#include "sam.h"


#define F_CPU 84000000UL
#define BAUD 9600UL
#define UBRR_VAL ((F_CPU)/(16* BAUD))


/*
 * Remember to update the Makefile with the (relative) path to the uart.c file.
 * This starter code will not compile until the UART file has been included in the Makefile. 
 * If you get somewhat cryptic errors referencing functions such as _sbrk, 
 * _close_r, _write_r, _fstat etc, you have most likely not done that correctly.

 * If you get errors such as "arm-none-eabi-gcc: no such file", you may need to reinstall the arm gcc packages using
 * apt or your favorite package manager.
 */
#include "uart.h"
#include "can.h"
#include "servo.h"
#include "encoder.h"
#include "diode.h"
#include "motor.h"
#include "solednoid.h"



int main()
{
    SystemInit();

    WDT->WDT_MR = WDT_MR_WDDIS; //Disable Watchdog Timer

    //Uncomment after including uart above
    uart_init(F_CPU, 9600);
    can_init((CanInit){
        .reg =  CAN_BR_BRP(13)
        |       CAN_BR_PROPAG(3)
        |       CAN_BR_PHASE1(3)
        |       CAN_BR_PHASE2(2)
        |       CAN_BR_SJW(0)       
    }, 0);

    CanMsg msg;

    printf("Hello nn\n\r");

    // Servo init
    servo_init();
    servo_set_pulse_width(1500u);
    servo_enable();
    encoder_init();
    diode_init();
    solenoid_init();
    motor_init();
    motor_set_position(10);
    uint8_t previousButton = 0;

    while (1)
    {
        //printf("Encoder count = %d\r\n", encoder_read());
        //printf("Diode readback: %d\r\n", diode_read());

        if (can_rx(&msg))
        {
            switch (msg.id)
            {
            case 0x101: // joystick
            printf("got joystick\r\n");
                uint8_t x = msg.byte[0];
                uint8_t y = msg.byte[1];
                uint8_t btn = msg.byte[2];
                //servo_set_pulse_width(1500u);
                //servo_set_pulse_width((uint16_t)x*12);

                if (x > 140) {
                motor_increase_position(1.5*(abs(x-128)));
                }
                if (x < 115) {
                    motor_decrease_position(1.5*(abs(x-128)));
                }
                if (btn != previousButton) {
                    solenoid_toggle();
                    previousButton = btn;
                } 
                break;
            case 0x102:
            printf("got slider\r\n");

                uint8_t slider_x = msg.byte[0];
                uint8_t touchpad_x = msg.byte[1];
                uint8_t touchpad_y = msg.byte[2];

                break;

            case 0x103:
            printf("got buton\r\n");

                uint8_t btn_r2 = msg.byte[0];
                break;
            
            case 0x100:
            printf("got solenoid\r\n");

                solenoid_toggle();

            default:
                break;
            }


            //can_printmsg(msg);
            can_tx(msg);
        }
        // motor_run_position() now runs from the TC0 interrupt at a fixed rate
        motor_print_status();
        /* code */
    }
    
}