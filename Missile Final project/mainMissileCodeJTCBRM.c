// File name: mainMissileCodeJTCBRM.c
// ECE230 Winter 2024-2025
/*! \file */
/******************************************************************************
 * MSP432 final project code
 *
 * Description: This is the main file for our final project, the IR-Seeking Missile.
 * This includes the control code for the IR sensor, stepper motor, servos, LED "ignition",
 * serial communication, and all other features.
 *
 * Author: John Tansey (JT), Christopher Bankson (CB), Ryan Mohseni (RM)
 *
 * Work on particular sections indicated by initials
 * Last-modified: 2/24/2025
 *
 *******************************************************************************/
 #include "msp.h"
 extern int steps;
 /* Standard Includes */
 #include <stdint.h>
 #include <stdbool.h>
 #include "csHFXT.h"
 #include "stepper.h"
 #include <stdio.h>
 #define LED1_Port P1
 #define LED1_Pin BIT0
 #define ANALOGCHANNEL 0x1 // A1
 #define AnalogA1_PORT P5
 #define AnalogA1_PIN BIT4
 #define MEMORYCHANNEL 0x1
 #define Threshold pow(2, 12) / 1.2 // 2047; 3.3/2=1.65V
 #include "msp.h"
 #include <math.h>
 #include "servoDriverTemplate_jjs.h"
 
 // Servo pins
 #define SERVO_1_PIN BIT6  // P5.6
 #define SERVO_2_PIN BIT7  // P5.7
 #define SERVO_3_PIN BIT6  // P6.6
 #define SERVO_4_PIN BIT7  // P6.7
 
 // Calibrated Servo pulse width ranges (in ticks) - JT, CB
 #define SERVO_1_MIN 4250
 #define SERVO_1_MAX 9000
 #define SERVO_2_MIN 4250
 #define SERVO_2_MAX 9500
 #define SERVO_3_MIN 4250
 #define SERVO_3_MAX 9750
 #define SERVO_4_MIN 4250
 #define SERVO_4_MAX 9000
 
 // ADC variables and thresholds - RM
 #define ADCThreshold 1500
 #define PI 3.14159265359
 #define ADC_THRESHOLD 4000
 #define ADC_ALERT 600
 
 // Serial commands setup - JT
 #define ARM_CMD "ARM"
 #define FIRE_CMD "FIRE"
 
 // Speaker, LED, payload pins and ports - JT
 #define SPEAKER_PORT P5
 #define LED_PORT P2
 #define PAYLOAD_PORT P5
 #define SPEAKER_PIN BIT6  // Pin 5.1
 #define LED_PIN BIT5      // Pin 2.5
 #define PAYLOAD_PIN BIT0  // Pin 5.0
 
 // Additional hardwired button inputs for added redundancy, should Serial communication fail whilst in use - JT
 #define BUTTON_ARM_PORT P4
 #define BUTTON_ARM_PIN BIT1  // P4.1
 #define BUTTON_FIRE_PORT P4
 #define BUTTON_FIRE_PIN BIT2  // P4.2
 
 // Initializing variables, prototypes for functions - JT
 static volatile uint16_t curADCResult;
 static volatile bool resultReady = false;
 volatile bool armed = false;
 volatile bool fire = false;
 volatile uint16_t adcResult = 0;
 void configureButtons(void);
 void debounce(void);
 bool isButtonPressed(uint8_t port, uint8_t pin);
/******************************************************************************
 * Function: configureSpeakerLEDPayload
 * Description: Configures the speaker, LED, and payload pins for output.
 * Author: JT
 ******************************************************************************/
 void configureSpeakerLEDPayload(void)
 {
     // Configure SPEAKER_PIN
     SPEAKER_PORT->SEL0 |= SPEAKER_PIN;
     SPEAKER_PORT->SEL1 &= ~SPEAKER_PIN;
     SPEAKER_PORT->DIR |= SPEAKER_PIN;
 
     LED_PORT->SEL0 &= ~LED_PIN;
     LED_PORT->SEL1 &= ~LED_PIN;
     LED_PORT->DIR |= LED_PIN;
 
     AnalogA1_PORT->SEL0 &= ~PAYLOAD_PIN;
     AnalogA1_PORT->SEL1 &= ~PAYLOAD_PIN;
     AnalogA1_PORT->DIR |= PAYLOAD_PIN;
 }
 
 /******************************************************************************
  * Function: calculateAngle
  * Description: Calculates the angle based on x and y coordinates.
  * Parameters:
  *   - x: X coordinate
  *   - y: Y coordinate
  * Returns: Calculated angle in degrees.
  * Author: JT, CB
  ******************************************************************************/
 float calculateAngle(float x, float y)
 {
     return atan2(y, x) * 180 / PI;  // Converts radians to degrees
 }
 
 /******************************************************************************
  * Function: ConfigureServo
  * Description: Configures all four servos according to the pins listed in definitions.
  * Author: JT, CB
  ******************************************************************************/
 void ConfigureServo(void)
 {
     // Configure P5.6 (Servo 1) and P5.7 (Servo 2) as outputs
     P5->SEL0 |= (SERVO_1_PIN | SERVO_2_PIN); // Set P5.6 and P5.7 to Timer_A2 function
     P5->SEL1 &= ~(SERVO_1_PIN | SERVO_2_PIN);
     P5->DIR |= (SERVO_1_PIN | SERVO_2_PIN);   // Set P5.6 and P5.7 as outputs
 
     // Configure P6.6 (Servo 3) and P6.7 (Servo 4) as outputs
     P6->SEL0 |= (SERVO_3_PIN | SERVO_4_PIN); // Set P6.6 and P6.7 to Timer_A2 function
     P6->SEL1 &= ~(SERVO_3_PIN | SERVO_4_PIN);
     P6->DIR |= (SERVO_3_PIN | SERVO_4_PIN);   // Set P6.6 and P6.7 as outputs
 
     // Configure Timer_A2 for PWM output
     TIMER_A2->CCR[0] = SERVO_TMR_PERIOD;  // Set period for 50Hz signal
 
     // Init servo positions to min pulse width
     TIMER_A2->CCR[1] = SERVO_1_MIN;  // Servo 1 (P5.6)
     TIMER_A2->CCR[2] = SERVO_2_MIN;  // Servo 2 (P5.7)
     TIMER_A2->CCR[3] = SERVO_3_MIN;  // Servo 3 (P6.6)
     TIMER_A2->CCR[4] = SERVO_4_MIN;  // Servo 4 (P6.7)
 
     // Configure Timer_A2 compare registers for PWM mode
     TIMER_A2->CCTL[1] = 0xE0;  // Reset/set mode for CCR1 (Servo 1)
     TIMER_A2->CCTL[2] = 0xE0;  // Reset/set mode for CCR2 (Servo 2)
     TIMER_A2->CCTL[3] = 0xE0;  // Reset/set mode for CCR3 (Servo 3)
     TIMER_A2->CCTL[4] = 0xE0;  // Reset/set mode for CCR4 (Servo 4)
 
     // Config Timer_A2 control register
     TIMER_A2->CTL = 0x02D0;  // SMCLK, up mode, divide by 1
     TIMER_A2->EX0 = 0x02;    // Divide by 3 for 16MHz timer clock
 }
 
 /******************************************************************************
  * Function: setServoAngle
  * Description: Sets the angle of a specific servo.
  * Parameters:
  *   - servo: Servo number (0-3)
  *   - angle: Desired angle in degrees
  * Author: JT
  ******************************************************************************/
 void setServoAngle(uint8_t servo, float angle)
 {
     float pulseWidthTicks;
     if (angle > 90)    // Angles are limited to avoid damage - JT
         angle = 90;
     if (angle < -90)
         angle = -90;
 
     if (curADCResult >= ADCThreshold)
     {
         // If ADC > 1500, move servo based on angle - JT
         switch (servo)
         {
         case 0:  // Servo 1 (P5.6)
             pulseWidthTicks = ((SERVO_1_MAX - SERVO_1_MIN) / 180.0)
                     * (angle + 90) + SERVO_1_MIN;
             TIMER_A2->CCR[1] = (uint16_t) pulseWidthTicks;
             break;
         case 1:  // Servo 2 (P5.7)
             pulseWidthTicks = ((SERVO_2_MAX - SERVO_2_MIN) / 180.0)
                     * (angle + 90) + SERVO_2_MIN;
             TIMER_A2->CCR[2] = (uint16_t) pulseWidthTicks;
             break;
         case 2:  // Servo 3 (P6.6)
             pulseWidthTicks = ((SERVO_3_MAX - SERVO_3_MIN) / 180.0)
                     * (angle + 90) + SERVO_3_MIN;
             TIMER_A2->CCR[3] = (uint16_t) pulseWidthTicks;
             break;
         case 3:  // Servo 4 (P6.7)
             pulseWidthTicks = ((SERVO_4_MAX - SERVO_4_MIN) / 180.0)
                     * (angle + 90) + SERVO_4_MIN;
             TIMER_A2->CCR[4] = (uint16_t) pulseWidthTicks;
             break;
         }
     }
     else
     {
         // If ADC below 1500, set servo to mid pos - JT
         switch (servo)
         {
         case 0:  // Servo 1 (P5.6)
             pulseWidthTicks = (SERVO_1_MIN + SERVO_1_MAX) / 2;
             TIMER_A2->CCR[1] = (uint16_t) pulseWidthTicks;
             break;
         case 1:  // Servo 2 (P5.7)
             pulseWidthTicks = (SERVO_2_MIN + SERVO_2_MAX) / 2;
             TIMER_A2->CCR[2] = (uint16_t) pulseWidthTicks;
             break;
         case 2:  // Servo 3 (P6.6)
             pulseWidthTicks = (SERVO_3_MIN + SERVO_3_MAX) / 2;
             TIMER_A2->CCR[3] = (uint16_t) pulseWidthTicks;
             break;
         case 3:  // Servo 4 (P6.7)
             pulseWidthTicks = (SERVO_4_MIN + SERVO_4_MAX) / 2;
             TIMER_A2->CCR[4] = (uint16_t) pulseWidthTicks;
             break;
         }
     }
 }
 
 /******************************************************************************
  * Function: configureUART
  * Description: Configures UART for serial communication with host device.
  * Author: JT
  ******************************************************************************/
 void configureUART(void)
 {
     P3->SEL0 |= BIT2 | BIT3;
     P3->SEL1 &= ~(BIT2 | BIT3);
 
     EUSCI_A2->CTLW0 |= EUSCI_A_CTLW0_SWRST;
     EUSCI_A2->CTLW0 = EUSCI_A_CTLW0_SWRST | EUSCI_A_CTLW0_SSEL__SMCLK;
     EUSCI_A2->BRW = 78;
     EUSCI_A2->MCTLW = (10 << EUSCI_A_MCTLW_BRF_OFS) | EUSCI_A_MCTLW_OS16
             | (0x44 << EUSCI_A_MCTLW_BRS_OFS);
 
     EUSCI_A2->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
     EUSCI_A2->IE |= EUSCI_A_IE_RXIE;
     NVIC->ISER[0] = (1 << EUSCIA2_IRQn);
     __enable_irq();
 }
 
 /******************************************************************************
  * Function: playTone
  * Description: Plays a tone on the speaker.
  * Parameters:
  *   - frequency: Frequency of the tone in Hz
  * Author: JT, CB
  ******************************************************************************/
 void playTone(uint16_t frequency)
 {
     uint16_t period = 48000000 / frequency;
     TIMER_A0->CCR[0] = period - 1;
     TIMER_A0->CCR[1] = (period / 2) - 1;
     TIMER_A0->CCTL[1] = 0x0060;
 }
 
 /******************************************************************************
  * Function: stopTone
  * Description: Stops any tone from playing on the speaker.
  * Author: JT
  ******************************************************************************/
 void stopTone(void)
 {
     TIMER_A0->CCTL[1] = 0x0000;
 }
 
 /******************************************************************************
  * Function: setFinAngles
  * Description: Sets the fin angles based on calculated coordinates.
  * Parameters:
  *   - x: X coordinate
  *   - y: Y coordinate
  * Author: JT
  ******************************************************************************/
 void setFinAngles(float x, float y)
 {
     float angles[4];
 
     angles[0] = calculateAngle(x, y);          // FIN A
     angles[1] = calculateAngle(-y, x);         // FIN B
     angles[2] = calculateAngle(-x, -y);        // FIN C
     angles[3] = calculateAngle(y, -x);         // FIN D
 
     // Map calculated angles to servo values and set each servo
     int inde = 0;
     for (inde = 0; inde < 4; inde++) // Loop iterates through each servo quickly
     {
         setServoAngle(inde, angles[inde]);
     }
 }
 
 /******************************************************************************
  * Function: configureButtons
  * Description: Configures the buttons for added redundancy.
  * Author: JT
  ******************************************************************************/
 void configureButtons(void)
 {
     // Configure ARM button (P4.1)
     BUTTON_ARM_PORT->SEL0 &= ~BUTTON_ARM_PIN;  // GPIO function
     BUTTON_ARM_PORT->SEL1 &= ~BUTTON_ARM_PIN;
     BUTTON_ARM_PORT->DIR &= ~BUTTON_ARM_PIN;   // Input
     BUTTON_ARM_PORT->REN |= BUTTON_ARM_PIN;    // Enable pull resistor
     BUTTON_ARM_PORT->OUT |= BUTTON_ARM_PIN;    // Pull-up
 
     // Configure FIRE button (P4.2)
     BUTTON_FIRE_PORT->SEL0 &= ~BUTTON_FIRE_PIN;  // GPIO function
     BUTTON_FIRE_PORT->SEL1 &= ~BUTTON_FIRE_PIN;
     BUTTON_FIRE_PORT->DIR &= ~BUTTON_FIRE_PIN;   // Input
     BUTTON_FIRE_PORT->REN |= BUTTON_FIRE_PIN;    // Enable pull resistor
     BUTTON_FIRE_PORT->OUT |= BUTTON_FIRE_PIN;    // Pull-up
 }
 
 /******************************************************************************
  * Function: debounce
  * Description: Simple debounce function for button presses.
  * Author: JT
  ******************************************************************************/
 void debounce(void)
 {
     volatile int delay = 1000;
     while (delay--);
     // Simple debounce delay
 }
 
 /******************************************************************************
  * Function: isButtonPressed
  * Description: Checks if a button is pressed.
  * Parameters:
  *   - port: Port of the button
  *   - pin: Pin of the button
  * Returns: True if the button is pressed, false otherwise.
  * Author: JT
  ******************************************************************************/
 bool isButtonPressed(uint8_t port, uint8_t pin)
 {
     if ((port & pin) == 0)
     {  // Button is active low
         debounce();
         if ((port & pin) == 0)
         {  // Confirm button press after debounce
             return true;
         }
     }
     return false;
 }
 
 /******************************************************************************
  * Function: main
  * Description: Main function for the IR-Seeking Missile project.
  * Author: JT, CB, RM
  ******************************************************************************/
int main(void)
{
    WDT_A->CTL = WDT_A_CTL_PW | WDT_A_CTL_HOLD;

    configHFXT();
    initStepperMotor();
    enableStepperMotor();
    ConfigureServo();
    configureSpeakerLEDPayload();
    configureButtons();

    volatile int i;
    float AnalogVoltage;

    // GPIO Setup Setup for LED pins - JT
    LED1_Port->SEL0 &= ~LED1_Pin;
    LED1_Port->SEL1 &= ~LED1_Pin;
    LED1_Port->OUT &= ~LED1_Pin;
    LED1_Port->DIR |= LED1_Pin;

    // Configure P5.4 for ADC (tertiary module function) - RM
    AnalogA1_PORT->DIR &= ~AnalogA1_PIN;
    AnalogA1_PORT->SEL0 = (AnalogA1_PORT->SEL0) | AnalogA1_PIN;
    AnalogA1_PORT->SEL1 = (AnalogA1_PORT->SEL1) | AnalogA1_PIN;

    ADC14->CTL0 = ADC14_CTL0_SHP | ADC14_CTL0_SHT0__16 | ADC14_CTL0_PDIV__1 // ADC setup - RM
            | ADC14_CTL0_DIV__1 | ADC14_CTL0_SHS_0 | ADC14_CTL0_SSEL__MODCLK
            | ADC14_CTL0_CONSEQ_0 | ADC14_CTL0_ON;
    ADC14->CTL1 = ADC14_CTL1_RES__12BIT
            | (MEMORYCHANNEL << ADC14_CTL1_CSTARTADD_OFS) | ADC14_CTL1_PWRMD_2;
    ADC14->MCTL[1] = 0x1;
    ADC14->IER0 = 0x2;
    NVIC->ISER[0] = (1 << ADC14_IRQn);
    __enable_irq();

    while (1)
    {
        // Check ARM button (P4.1)
        if (isButtonPressed(BUTTON_ARM_PORT->IN, BUTTON_ARM_PIN))   // check buttons, update states - JT
        {
            armed = !armed;  // Toggle armed state
            while (isButtonPressed(BUTTON_ARM_PORT->IN, BUTTON_ARM_PIN))
                ;  // Wait for button release
        }

        // Check FIRE button (P4.2)
        if (isButtonPressed(BUTTON_FIRE_PORT->IN, BUTTON_FIRE_PIN)) // check buttons, update states - JT
        {
            fire = !fire;  // Toggle fire state
            while (isButtonPressed(BUTTON_FIRE_PORT->IN, BUTTON_FIRE_PIN))
                ;  // Wait for button release
        }

        // stepper code - RM
        int localSteps = steps;
        bool detect = 0;

        for (localSteps = steps; localSteps % 16 != 0; localSteps = steps)
        {
            // Wait until steps are a multiple of 16
        }

        bool directionforward = localSteps % (REVERSE_STEPS * 2) < REVERSE_STEPS;
        int progress = (localSteps % REVERSE_STEPS);
        if (!directionforward)
        {
            progress = REVERSE_STEPS - progress;
        }

        float angledegrees = (360.0f * progress) / REVERSE_STEPS;

        // ADC code - RM

        // Enable and start sampling/conversion by ADC
        ADC14->CTL0 = ADC14->CTL0 | 0x3;

        if (curADCResult >= ADCThreshold)
            detect = 1;
        else
            detect = 0;

        AnalogVoltage = (float) curADCResult * 3.3 / pow(2, 12);

        // Calculate x and y based on angledegrees - JT
        float x = cos(angledegrees * PI / 180.0);
        float y = sin(angledegrees * PI / 180.0);

        // Set fin angles based on calculated coordinates - JT
        setFinAngles(x, y);

        // Control code for speakers and for payload and led "ignition" - JT
        if (armed)
        {
            playTone(800);
            if (adcResult > ADC_ALERT)
                playTone(800);
            if (fire && adcResult >= ADC_THRESHOLD)
            {
                P3->OUT |= PAYLOAD_PIN;
                P1->OUT |= LED_PIN;
            }
        }
        else
        {
            stopTone();
        }
    }
}
/******************************************************************************
 * Function: ADC14_IRQHandler
 * Description: Interrupt handler for ADC14. Checks if the interrupt is triggered
 *              by ADC14MEM1 conversion value loaded and updates the LED state
 *              based on the ADC result.
 * Author: RM
 ******************************************************************************/
 void ADC14_IRQHandler(void)
 {
     // Check if interrupt triggered by ADC14MEM1 conversion value loaded
     // Not necessary for this example since only one ADC channel is used
     if (ADC14->IFGR0 & ADC14_IFGR0_IFG1)
     {
         curADCResult = ADC14->MEM[MEMORYCHANNEL];
         if (curADCResult >= Threshold)  // ADC12MEM1 = A1 > 0.5AVcc?
             LED1_Port->OUT |= LED1_Pin;  // Turn LED1 on
         else
             LED1_Port->OUT &= ~LED1_Pin; // Turn LED1 off
         resultReady = true;
         // Not necessary to clear flag because reading ADC14MEMx clears the flag
     }
 }
 
 /******************************************************************************
  * Function: EUSCIA2_IRQHandler
  * Description: Interrupt handler for serial communication via UART. Processes
  *              incoming serial commands and updates the armed and fire states
  *              based on the received commands.
  * Author: JT
  ******************************************************************************/
 void EUSCIA2_IRQHandler(void)
 {
     if (EUSCI_A2->IFG & EUSCI_A_IFG_RXIFG)
     {
         static char buffer[5];
         static uint8_t index = 0;
         char receivedChar = EUSCI_A2->RXBUF;
         if (receivedChar == '\n' || receivedChar == '\r')
         {
             buffer[index] = '\0';
             if (strcmp(buffer, ARM_CMD) == 0)
                 armed = true;
             if (strcmp(buffer, FIRE_CMD) == 0)
                 fire = true;
             index = 0;
         }
         else if (index < sizeof(buffer) - 1)
         {
             buffer[index++] = receivedChar;
         }
     }
 }