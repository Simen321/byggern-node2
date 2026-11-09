#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
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


#include "can_ids.h"


// ---- ir beam / game ----
#define diode_threshold 1500        // below this the beam is blocked
#define beam_block_ticks 6          // beam must be blocked this long (6 * 5 ms = 30 ms) to count, filters noise

uint8_t gameRunning = 0;            // set by CAN_ID_GAME_START from node 1
uint8_t beamBlocked = 0;
uint32_t beamBlockedSince = 0;


// ---- console log to node 1 ----
// text is queued here and sent as CAN_ID_LOG frames, one frame per log_frame_ticks.
// node 1 only has two receive buffers and checks them once per screen update, so
// sending many frames back to back would overflow it.
#define log_frame_ticks 5           // 5 * 5 ms = 25 ms between frames
#define LOG_QUEUE_SIZE 256          // must be a power of two
static char logQueue[LOG_QUEUE_SIZE];
static uint16_t logHead = 0;
static uint16_t logTail = 0;
static uint32_t lastLogFrame = 0;

// queue a line of text for the console on node 1. a '\n' is added at the end
void log_send(const char *text) {
    while (1) {
        char c = *text ? *text++ : '\n';
        uint16_t next = (logHead + 1) & (LOG_QUEUE_SIZE - 1);
        if (next == logTail) {
            return; // queue full, drop the rest
        }
        logQueue[logHead] = c;
        logHead = next;
        if (c == '\n') {
            return;
        }
    }
}

// send at most one log frame, call every loop
void log_task(void) {
    if (logTail == logHead || motor_get_ticks() - lastLogFrame < log_frame_ticks) {
        return;
    }
    CanMsg m = {.id = CAN_ID_LOG, .length = 0};
    while (m.length < 8 && logTail != logHead) {
        char c = logQueue[logTail];
        logTail = (logTail + 1) & (LOG_QUEUE_SIZE - 1);
        m.byte[m.length++] = c;
        if (c == '\n') {
            break; // one line per frame at most, keeps lines easy to split on node 1
        }
    }
    can_tx(m);
    lastLogFrame = motor_get_ticks();
}


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
    }, 1); // 1: receive with interrupt into a queue, so no frames are lost

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

    log_send("node 2 started");

    while (1)
    {
        //printf("Encoder count = %d\r\n", encoder_read());
        //printf("Diode readback: %d\r\n", diode_read());

        // handle every frame that has arrived since last loop
        while (can_rx(&msg))
        {
            switch (msg.id)
            {
            case CAN_ID_JOYSTICK:
            {
                uint8_t x = msg.byte[0];
                uint8_t y = msg.byte[1];
                uint8_t btn = msg.byte[2];

                if (x > 140) {
                    motor_increase_position(1.5*(abs(x-128)));
                }
                if (x < 115) {
                    motor_decrease_position(1.5*(abs(x-128)));
                }
                // fire only when the button goes down, not again when it is released
                if (btn && !previousButton) {
                    solenoid_toggle();
                }
                previousButton = btn;
                break;
            }
            case CAN_ID_SLIDER:
            {
                uint8_t slider_x = msg.byte[0];
                uint8_t touchpad_x = msg.byte[1];
                uint8_t touchpad_y = msg.byte[2];

                // multiply before dividing, otherwise 2200/255 becomes 8 and max is 8*255 = 2040
                motor_set_position((int)slider_x * 2200 / 255);
                servo_set_pulse_width((int)touchpad_x * 1200 / 255 + 900);
                break;
            }
            case CAN_ID_BUTTONS:
            {
                uint8_t btn_r2 = msg.byte[0];
                break;
            }
            case CAN_ID_SOLENOID:
                solenoid_toggle();
                break;

            case CAN_ID_GAME_START:
                gameRunning = 1;
                beamBlocked = 0;
                log_send("game started");
                break;

            case CAN_ID_GAME_STOP:
                gameRunning = 0;
                log_send("game stopped");
                break;

            default:
                break;
            }
        }

        // ball lost: the beam has been blocked for beam_block_ticks during a game
        uint32_t now = motor_get_ticks();
        if (diode_read() < diode_threshold) {
            if (!beamBlocked) {
                beamBlocked = 1;
                beamBlockedSince = now;
            }
        } else {
            beamBlocked = 0;
        }
        if (gameRunning && beamBlocked && (now - beamBlockedSince >= beam_block_ticks)) {
            gameRunning = 0;
            can_tx((CanMsg){.id = CAN_ID_GAME_OVER, .length = 0});
            log_send("ball lost");
        }

        log_task();

        // motor_run_position() now runs from the TC0 interrupt at a fixed rate
        motor_print_status();
    }
    
}
