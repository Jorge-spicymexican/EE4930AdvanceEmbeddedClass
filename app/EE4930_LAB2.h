/*
 * EE4930_LAB2.h
 *
 *  Created on: Dec 1, 2022
 *      Author: jurado-garciaj
 *  Specifications:
 *      Perform A/D conversion on a analog input with Potentiometer
 *      A/D specs:
 *          Single-channel single-conversion mode
 *          10-bit resolution
 *          Use a timer and its interrupt to start a new A/D conversion
 *          Use A/D interrupt for handling of new conversion values
 *      Read A/D reading display the raw reading and duty cycle on LCD when SW1 is pressed
 *      Use an Interrupt with P1.1 to start this action.
 *
 *      Use a Timer to generate a PWM signal with a duty cycle being the percentage inverted
 *          if the input is at 20%, set the duty cycle to 80%,
 *          if at 0%, set the duty cycle to 30%,
 *
 *      Use a Scope to see the PWM wave or a LED as a visual indication
 *
 * Initialize GPIO. Setup inputs and outputs as follows:
 *
 *  Port pin    in/out  Pullup/down     Connect
 *    P1.1        In       UP             s1
 *    P5.5        IN       UP             Pot
 *    P6.6       Out     N/A             Logic Pro
 *
 *
************   Nokia LCD interface reference   **************
//
// Red SparkFun Nokia 5110 (LCD-10168)
// -----------------------------------
// Signal        (Nokia 5110) LaunchPad pin
// 3.3V          (VCC, pin 1) power
// Ground        (GND, pin 2) ground
// UCA3STE       (SCE, pin 3) connected to P9.4
// Reset         (RST, pin 4) connected to P9.3
// Data/Command  (D/C, pin 5) connected to P9.2
// UCA3SIMO      (DN,  pin 6) connected to P9.7
// UCA3CLK       (SCLK, pin 7) connected to P9.5
// back light    (LED, pin 8) not connected, consists of 4 3.3 V white LEDs which draw ~80mA total
 *
 *
 */

#ifndef APP_EE4930_LAB2_H_
#define APP_EE4930_LAB2_H_


extern void Init_Lab2( void );

extern int Set_Duty(float duty);



#endif /* APP_EE4930_LAB2_H_ */
