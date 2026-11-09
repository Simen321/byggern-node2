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


// ---- control settings from the settings menu on node 1 ----
uint8_t controlScheme = CONTROL_POSITION;
int maxSpeedSetting = 0;            // 0 = not received yet, the motor uses its own default
uint16_t pidSetting[3] = {0, 0, 0}; // p, i, d in hundredths, 0 = not received yet

// the slider and touchpad are only followed after they have been moved, so the motor and servo
// stay in the middle at start (and when switching scheme) until someone touches them
#define touch_threshold 3           // change needed before we start following
#define joystick_divider 4          // velocity control: target moves |x-128|/4 ticks per message (60 per second)
int sliderStart = -1;               // first value seen, -1 = none yet
uint8_t sliderFollowing = 0;
int touchStart = -1;
uint8_t touchFollowing = 0;

// follow `value` once it has moved more than touch_threshold away from the first value seen
uint8_t has_been_touched(uint8_t value, int *start, uint8_t *following) {
    if (*start < 0) {
        *start = value;
    }
    if (abs(value - *start) > touch_threshold) {
        *following = 1;
    }
    return *following;
}


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
    motor_set_position(motor_max_position / 2); // start in the middle
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

                // velocity control: the further the joystick is pushed, the faster the target moves
                if (controlScheme == CONTROL_VELOCITY) {
                    if (x > 140) {
                        motor_increase_position(abs(x-128) / joystick_divider);
                    }
                    if (x < 115) {
                        motor_decrease_position(abs(x-128) / joystick_divider);
                    }
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

                // position control: the slider sets the position
                // multiply before dividing, otherwise 2200/255 becomes 8 and max is 8*255 = 2040
                if (controlScheme == CONTROL_POSITION &&
                    has_been_touched(slider_x, &sliderStart, &sliderFollowing)) {
                    motor_set_position((int)slider_x * motor_max_position / 255);
                }
                if (has_been_touched(touchpad_x, &touchStart, &touchFollowing)) {
                    servo_set_pulse_width((int)touchpad_x * 1200 / 255 + 900);
                }
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

            // node 1 sends the settings when they change and every few seconds,
            // so only act (and log) when something is different
            case CAN_ID_SETTINGS_CONTROL:
            {
                uint8_t scheme = msg.byte[0];
                int maxSpeed = (msg.byte[1] << 8) | msg.byte[2];
                if (scheme != controlScheme || maxSpeed != maxSpeedSetting) {
                    if (scheme != controlScheme) {
                        // wait for the slider to be moved again before following it
                        sliderStart = -1;
                        sliderFollowing = 0;
                    }
                    controlScheme = scheme;
                    maxSpeedSetting = maxSpeed;
                    motor_set_max_speed(maxSpeed);

                    char text[24];
                    snprintf(text, sizeof(text), "%s, max %d",
                             scheme == CONTROL_VELOCITY ? "velocity" : "position", maxSpeed);
                    log_send(text);
                }
                break;
            }
            case CAN_ID_SETTINGS_PID:
            {
                uint16_t pid[3];
                for (int i = 0; i < 3; i++) {
                    pid[i] = (msg.byte[2 * i] << 8) | msg.byte[2 * i + 1];
                }
                if (pid[0] != pidSetting[0] || pid[1] != pidSetting[1] || pid[2] != pidSetting[2]) {
                    for (int i = 0; i < 3; i++) {
                        pidSetting[i] = pid[i];
                    }
                    motor_set_pid(pid[0] / 100.0f, pid[1] / 100.0f, pid[2] / 100.0f);

                    char text[24];
                    snprintf(text, sizeof(text), "PID %u.%02u %u.%02u %u.%02u",
                             pid[0] / 100, pid[0] % 100, pid[1] / 100, pid[1] % 100, pid[2] / 100, pid[2] % 100);
                    log_send(text);
                }
                break;
            }

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
