/*! \file */
/*!
 * stepper.c
 * ECE230 Winter 2024-2025
 *
 * Description: Modified from Stepper motor ULN2003 driver for MSP432P4111 Launchpad.
 *              Assumes SMCLK configured with 48MHz HFXT as source.
 *              Uses Timer_A3 and P2.7, P2.6, P2.5, P2.4.asm
 *              Modified to use on a bipolar stepper driver
 *
 *  Last modified on:   2/24/2025
 *      Author: Jinjian Song, Ryan Mohseni, John Tansey
 */

#include <stepper.h>
#include "msp.h"
int steps = 0;

uint16_t stepPeriod = INIT_PERIOD;


void initStepperMotor(void) {
    // set stepper port pins as GPIO outputs
    STEPPER_PORT->SEL0 = (STEPPER_PORT->SEL0) & ~STEPPER_MASK;
    STEPPER_PORT->SEL1 = (STEPPER_PORT->SEL1) & ~STEPPER_MASK;
    STEPPER_PORT->DIR = (STEPPER_PORT->DIR) | STEPPER_MASK;

    // initialize stepper outputs to LOW
    STEPPER_PORT->OUT = (STEPPER_PORT->OUT) & ~STEPPER_MASK;

    /* Configure Timer_A3 and CCR0 */
    // Set period of Timer_A3 in CCR0 register for Up Mode
    TIMER_A3->CCR[0] = stepPeriod;
    //configure CCR0 for Compare mode with interrupt enabled (no output mode - 0)
    TIMER_A3->CCTL[0] = 0x10;
    // Configure Timer_A3 in Stop Mode, with source SMCLK, prescale 12:1, and
    //  interrupt disabled  -  tick rate will be 4MHz (for SMCLK = 48MHz)
    // configure Timer_A3 (requires setting control AND expansion register)
    TIMER_A3->CTL =0x240;
    TIMER_A3->EX0 =0x5;

    /* Configure global interrupts and NVIC */
    // Enable TA3CCR0 compare interrupt by setting IRQ bit in NVIC ISER0 register
    // enable interrupt by setting IRQ bit in NVIC ISER0 register
    NVIC->ISER[0] |= (1 << (TA3_N_IRQn-1));
    __enable_irq();                             // Enable global interrupt
}

void enableStepperMotor(void) {
    // configure Timer_A3 in Up Mode (leaving remaining configuration unchanged)
    TIMER_A3->CTL =0x250;
}

void disableStepperMotor(void) {
    TIMER_A3->CTL =0x240;
    //  Configure Timer_A3 in Stop Mode (leaving remaining configuration unchanged)
}


void stepClockwise(void) {
    steps++;
    static unsigned char set = 0;

    int temp = ((steps % (REVERSE_STEPS*2)) >= REVERSE_STEPS) ? 1 : 0;
    set = (STEPPER_PORT->OUT & 0xF) + (temp << 7) + ((steps & 1) << 4);
    STEPPER_PORT->OUT = set;
}

// Timer A3 CCR0 interrupt service routine
void TA3_0_IRQHandler(void)
{
    stepClockwise();
    //  clear timer compare flag in TA3CCTL0
    TIMER_A3->CCTL[0]&= ~0x01;

}
