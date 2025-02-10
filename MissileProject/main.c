#include "msp.h"
#include <math.h>
#include "servoDriverTemplate_jjs.h"

#define PI 3.14159265359

// from project 4
void ConfigureServo(void) {

    ServoControl_Port->SEL0 = (ServoControl_Port->SEL0) |= ServoControl_Pin;
    ServoControl_Port->SEL1 = (ServoControl_Port->SEL1) &= ~ServoControl_Pin;
    ServoControl_Port->DIR = (ServoControl_Port->DIR) |= ServoControl_Pin;
    ServoControl_Port->OUT = (ServoControl_Port->OUT) &= ~ServoControl_Pin;

    TIMER_A2->CCR[0] = SERVO_TMR_PERIOD;
    TIMER_A2->CCR[1] = SERVO_MIN_ANGLE; // Default FIN A
    TIMER_A2->CCR[2] = SERVO_MIN_ANGLE; // Default FIN B
    TIMER_A2->CCR[3] = SERVO_MIN_ANGLE; // Default FIN C
    TIMER_A2->CCR[4] = SERVO_MIN_ANGLE; // Default FIN D

    TIMER_A2->CCTL[1] = 0xE0;// fin a
    TIMER_A2->CCTL[2] = 0xE0; // fin b
    TIMER_A2->CCTL[3] = 0xE0; // fin c
    TIMER_A2->CCTL[4] = 0xE0;// fin d
    TIMER_A2->CTL = 0x2D0;
    TIMER_A2->EX0=0x02;
}

// calculate angles based on x and y
float calculateAngle(float x, float y) {
    return atan2(y, x) * 180 / PI;  // convert rad to deg
}

// set servo angle
void setServoAngle(uint8_t servo, float angle) {
    float pulseWidthTicks;
    // make sure angle is between -90 to 90 degrees
    if (angle > 90) angle = 70;
    if (angle < -90) angle = -70;
    // Map angle to pulse width ticks
    pulseWidthTicks = ((SERVO_MAX_ANGLE - SERVO_MIN_ANGLE) / 180.0) * (angle + 90) + SERVO_MIN_ANGLE;
    // Set the CCR1 register for Timer_A2
    switch (servo) {
            case 0:
                TIMER_A2->CCR[1] = (uint16_t)pulseWidthTicks; // FIN A
                break;
            case 1:
                TIMER_A2->CCR[2] = (uint16_t)pulseWidthTicks; // FIN B
                break;
            case 2:
                TIMER_A2->CCR[3] = (uint16_t)pulseWidthTicks; // FIN C
                break;
            case 3:
                TIMER_A2->CCR[4] = (uint16_t)pulseWidthTicks; // FIN D
                break;
        }
}

// set fin angles based on calculated coordinates
void setFinAngles(float x, float y) {
    float angles[4];

    angles[0] = calculateAngle(x, y);          // FIN A
    angles[1] = calculateAngle(-y, x);         // FIN B
    angles[2] = calculateAngle(-x, -y);        // FIN C
    angles[3] = calculateAngle(y, -x);         // FIN D

    // Map calculated angles to servo values and set each servo
    int inde = 0;
    for (inde = 0; inde < 4; inde++) {
        setServoAngle(inde, angles[inde]);
    }
}

int main(void) {
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    ConfigureServo();

    float x, y;

    while (1) {
        // ryans thing will update values in here
        x = 1.0;
        y = 1.0; // temp vals
        setFinAngles(x, y);
        __delay_cycles(48000000); // Add delay for test
    }
}
