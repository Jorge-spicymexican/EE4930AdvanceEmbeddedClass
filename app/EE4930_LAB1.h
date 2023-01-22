/*
 * EE4930_LAB1.h
 *
 *  Created on: Dec 1, 2022
 *      Author: jurado-garciaj
 * Initialize GPIO. Setup inputs and outputs as follows:
 *
 *  Port pin    in/out  Pullup/down     Connect
 *    P1.1        In       UP              s1
 *    P2.2        Out     N/A             LED2-B
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

#ifndef APP_EE4930_LAB1_H_
#define APP_EE4930_LAB1_H_


extern void Init_Lab1( void );

extern void Lab1_Poll( void );

#endif /* APP_EE4930_LAB1_H_ */
