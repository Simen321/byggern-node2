#include "motor.h"
#include <sam.h>
#include <stdlib.h>
#include <math.h>


// controller gains. error is in encoder ticks, output is in PWM duty counts (CPRD = 4200)
// these are the start values, the settings menu on node 1 can change them (motor_set_pid)
volatile float k_p = 5.0f;      // duty per tick of error
volatile float k_i = 1.0f;      // duty per tick*second of error
volatile float k_d = 0.25f;     // duty per tick/second of velocity

#define i_term_limit 800.0f     // anti-windup: the I term alone can never give more than this

#define control_hz motor_tick_hz // 200, set in motor.h
#define dt (1.0f / control_hz)

// friction compensation. the motor does not move below about 700, so that is our new zero.
// measure these in each direction and set them just below where it starts moving.
#define min_speed_dir0 400
#define min_speed_dir1 400
volatile int max_speed = 2500;  // highest PWM duty the controller may use, changed with motor_set_max_speed

// homing at start: drive towards 0 until the motor stops against the end, then call that 0
#define homing_speed 1000           // PWM duty, has to be above min_speed to move at all
#define homing_step_loops 120000    // busy-wait loops between encoder checks, roughly 10 ms
#define homing_still_steps 20       // encoder unchanged for this many checks (~200 ms) means we hit the end
#define homing_max_steps 600        // give up after ~6 s

#define position_tolerance 10   // ticks. inside this we stop and clear the integral
#define max_ramp_step 30        // ticks per control tick the reference may move (3000 ticks/s)

#define maxPosition motor_max_position // 2200, set in motor.h
#define minPosition 0


volatile int wantedPosition = 0;    // set from main loop (joystick)
volatile int referencePosition = 0; // ramps toward wantedPosition inside the controller
uint8_t previousDirection = 1;

float iTerm = 0;
int previousPosition = 0;

// debug values, written in the interrupt and printed from the main loop
volatile uint32_t controlTicks = 0;
volatile int debugPosition = 0;
volatile int debugEffort = 0;
volatile int debugSpeed = 0;



void motor_init(void) {

    // arduino 20, but in sam: PB12.
    
    // enable pmc clock on pwm peripheral
    PMC->PMC_PCER1 = (1u << (ID_PWM - 32)); // -32 since PCER1 controls 32-63, becomes 4.

    PIOB->PIO_ABSR |= (1u << 12); // set PB12 to peripherial type B.
    PIOB->PIO_PDR   = (1u << 12); // disable GPIO on pin.

    // direction pin: on arduino 7, but in sam: PC23
    PIOC->PIO_PER   = (1u << 23); // set to gpio 
    PIOC->PIO_CODR = (1u << 23); // low
    PIOC->PIO_OER = (1u << 23); // set it to be output

    PWM->PWM_DIS = (1u << 0); //OFF

    PWM->PWM_CH_NUM[0].PWM_CMR = (1u << 9); // 84Mhz, CPOL=1 so higher pwm is higher motor speed

    // 20khz = 84mgz/x, 4200
    PWM->PWM_CH_NUM[0].PWM_CPRD = 4200u;  
    PWM->PWM_CH_NUM[0].PWM_CDTY = 0;

    PWM->PWM_ENA = (1u << 0); // ENABLE

    // the encoder only counts how far we moved since power on, so find the end first
    motor_home();

    previousPosition = encoder_read();
    referencePosition = previousPosition;

    // TC0 because TC2 is used in encoder. had to add when we found that delta time varied
    // without a hardware timer on it
    // TC0 channel 0 gives the fixed control rate. MCK/128 = 656250 Hz, interrupt on RC compare


    PMC->PMC_PCER0 = (1u << ID_TC0); // turn the TC0 timer on
    TC0->TC_CHANNEL[0].TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK4 | TC_CMR_WAVE | TC_CMR_WAVSEL_UP_RC; // set
    TC0->TC_CHANNEL[0].TC_RC = (84000000u / 128u) / control_hz;
    TC0->TC_CHANNEL[0].TC_IER = TC_IER_CPCS; // interupt
    NVIC_SetPriority(TC0_IRQn, 2); // lower priority than CAN (0), so CAN frames are not lost while this runs
    NVIC_EnableIRQ(TC0_IRQn);
    TC0->TC_CHANNEL[0].TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG;
}


// drive slowly towards 0 (the cable side) until the encoder stops changing, which means the
// motor is against the end. then reset the encoder so that end is position 0.
// runs before the control interrupt is started, so it uses busy-waiting.
void motor_home(void) {
    motor_set_direction(0); // direction 0 moves towards position 0
    PWM->PWM_CH_NUM[0].PWM_CDTYUPD = homing_speed;

    int lastPosition = encoder_read();
    int stillSteps = 0;
    for (int step = 0; step < homing_max_steps && stillSteps < homing_still_steps; step++) {
        for (volatile int i = 0; i < homing_step_loops; i++) {}

        int position = encoder_read();
        if (abs(position - lastPosition) <= 1) {
            stillSteps++;
        } else {
            stillSteps = 0;
        }
        lastPosition = position;
    }

    PWM->PWM_CH_NUM[0].PWM_CDTYUPD = 0;
    for (volatile int i = 0; i < homing_step_loops * 10; i++) {} // let it settle, ~100 ms
    encoder_reset();
}


void TC0_Handler(void) {
    TC0->TC_CHANNEL[0].TC_SR; // reading SR clears the interrupt flag
    motor_run_position();
}



void motor_set_direction(uint8_t direction) {
    // disable if swapping direction
    if (direction != previousDirection){
        PWM->PWM_CH_NUM[0].PWM_CDTYUPD = 0;
        volatile int i = 25000;
        while (i >= 0) {
            i--;
        }
        previousDirection = direction;
    }

    if (direction == 0) {
        PIOC->PIO_SODR = (1u << 23);
    } else {
        PIOC->PIO_CODR = (1u << 23);
    }
}

void motor_set_speed(uint16_t speed) {
    PWM->PWM_CH_NUM[0].PWM_CDTYUPD = min(speed, max_speed);
}



// 0 is towards the cable side of the box.
// clamps to the allowed range instead of ignoring values outside it, so the ends can be reached
void motor_set_position(int position) {
    position = min(position, maxPosition);
    position = max(position, minPosition);
    wantedPosition = position;
}

void motor_increase_position(uint8_t increment) {
    motor_set_position(wantedPosition + increment);
}
void motor_decrease_position(uint8_t increment) {
    motor_set_position(wantedPosition - increment);
}


// runs at control_hz from TC0_Handler. no printf in here, it would block the interrupt
void motor_run_position(void) {
    int currentPosition = encoder_read();
    controlTicks++;

    // move the reference toward the wanted position at a limited rate
    int step = wantedPosition - referencePosition;
    step = min(step, max_ramp_step);
    step = max(step, -max_ramp_step);
    referencePosition += step;

    int error = currentPosition - referencePosition;

    // derivative on measurement, so setpoint changes do not kick the D term
    float velocity = (currentPosition - previousPosition) / dt;
    previousPosition = currentPosition;

    debugPosition = currentPosition;

    // close enough: stop and clear the integral so it does not wind up while standing still
    if (abs(error) < position_tolerance) {
        iTerm = 0;
        debugEffort = 0;
        debugSpeed = 0;
        motor_set_speed(0);
        return;
    }

    float effort = k_p * error + iTerm + k_d * velocity;
    uint8_t direction = effort < 0;

    // friction compensation: start counting from where the motor actually moves
    int minSpeed = direction ? min_speed_dir1 : min_speed_dir0;
    int speed = minSpeed + (int)fabsf(effort);

    // conditional integration: only integrate when the output is not saturated
    if (speed < max_speed) {
        iTerm += k_i * error * dt;
        iTerm = min(iTerm, i_term_limit);
        iTerm = max(iTerm, -i_term_limit);
    }
    speed = min(speed, max_speed);

    debugEffort = (int)effort;
    debugSpeed = speed;

    motor_set_direction(direction);
    motor_set_speed(speed);
}


// new controller gains from the settings menu. the integral is cleared so the old one does not kick
void motor_set_pid(float p, float i, float d) {
    k_p = p;
    k_i = i;
    k_d = d;
    iTerm = 0;
}

// highest PWM duty the controller may use, between the friction limit and the full period (4200)
void motor_set_max_speed(int speed) {
    speed = max(speed, min_speed_dir0 + 100);
    speed = min(speed, 4200);
    max_speed = speed;
}


// number of control ticks since start (control_hz per second). used as a clock by main
uint32_t motor_get_ticks(void) {
    return controlTicks;
}


void motor_print_status(void) {
    static uint32_t lastPrint = 0;
    if (controlTicks - lastPrint >= control_hz / 2) {
        lastPrint = controlTicks;
        printf("wanted: %d, ref: %d, pos: %d, effort: %d, speed: %d\r\n",
               wantedPosition, referencePosition, debugPosition, debugEffort, debugSpeed);
    }
}
