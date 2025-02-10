/*! \file */
/*!
 * servoDriverTemplate_jjs.h
 * ECE230-01/02 Winter 2023-2024
 * Date: January 11, 2024
 *
 * Description: Servo motor driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A2 and P5.6 (TA2.1)
 *
 **                 MSP432P4111
 *             -------------------
 *         /|\|                   |
 *          | |                   |
 *          --|RST                |
 *            |      (TA2.1) P5.6 |----> Servo
 *            |                   |
 *            |              PJ.2 |------
 *       S1-->|P1.1               |     |
 *            |                   |    HFXT @ 48MHz
 *            |                   |     |
 *            |              PJ.3 |------
 *  Created on:
 *      Author:
 */

#ifndef SERVODRIVER_H_
#define SERVODRIVER_H_

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include "msp.h"

//Servo control pin
#define ServoControl_Port P5
#define ServoControl_Pin BIT6

#define SMCLK 48000000 //Hz
#define TimerA0Prescaler 16 //Timer A prescaler
#define TimerA2Clock  SMCLK/TimerA0Prescaler
#define SERVO_FREQUENCY 50  //Hz
#define SERVO_MINIMUM   750 //us
#define SERVO_MAXIMUM   2250    //us

// TODO add tick count values for constants
#define SERVO_TMR_PERIOD                50000        // ticks for 50Hz signal
#define SERVO_MIN_ANGLE                 2250/1.5        // ticks for 0.75ms pulse
#define SERVO_MAX_ANGLE                 6750/1.5        // ticks for 2.25ms pulse
#define TEN_DEGREE_TICKS      (SERVO_MAX_ANGLE-SERVO_MIN_ANGLE)/18        // 250, ticks 10 degree shift


/*!
 * \brief This function configures pins and timer for servo motor driver
 *
 * This function configures P5.6 as output pin for servo drive signal and
 *  initializes Timer_A2 CCR1 for PWM output
 *
 * Modified bit 6 of \b P2DIR register and \b P2SEL registers.
 * Modified \b TA2CTL, \b TA2CCTL1 and CCR registers.
 *
 * \return None
 */
extern void ConfigureServo(void);


/*!
 * \brief This increments servo angle 10 degrees, with wrap-around
 *
 * This function increments servo angle by 10 degrees. If new angle exceeds max
 *  angle (+90), it wraps around to min angle (-90)
 *
 * Modified \b TA2CCR1 register.
 *
 * \return None
 */
extern void incrementTenDegree(void);


/*!
 * \brief This function sets angle of servo
 *
 * This function sets angle of servo to \a angle (between -90 to 90)
 *
 *  \param angle Angle in degrees to set servo (between -90 to 90)
 *
 * Modified \b TA2CCR1 register.
 *
 * \return None
 */

extern void setServoAngle(uint8_t servo, float rpm);


extern void ResetServoAngle(uint16_t rpm);

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif /* SERVODRIVER_H_ */
