/*
 * EE4930_LAB6.h
 *
 *  Created on: Feb 2, 2023
 *      Author: jurado-garciaj
 *
 *  Description:
 *  Reads from a potentiometer with an ADC. ADC reading is
 *  periodically taken using a TI RTOS clock
 *
 *********   Nokia LCD interface reference   **********************************
 *           Red SparkFun Nokia 5110 (LCD-10168)
 *           -----------------------------------
 *           Signal        (Nokia 5110) LaunchPad pin
 *           3.3V          (VCC, pin 1) power
 *           Ground        (GND, pin 2) ground
 *           UCA3STE       (SCE, pin 3) connected to P9.4
 *           Reset         (RST, pin 4) connected to P9.3
 *           Data/Command  (D/C, pin 5) connected to P9.2
 *           UCA3SIMO      (DN,  pin 6) connected to P9.7
 *           UCA3CLK       (SCLK, pin 7) connected to P9.5
 *           back light    (LED, pin 8) not connected
 *
 *           POTENTIOMETER reference
 *           GND    (CCW,   pin 1)  ground
 *           Wiper  (Wiper, pin 2)  connected to
 *           VCC    (CW,    pin 3)  power
 *
 */

#ifndef APP_EE4930_LAB6_H_
#define APP_EE4930_LAB6_H_

#include <xdc/std.h>

#define CLOCK_PERIOD 10
#define RTOS_ADC_INTERRUPT_NUMBER 40
#define ADC_12_BIT_RES (102)
#define TASKSTACKSIZE   512


//Init the lcd
void lcd_init( void );
void adc_init( void );
void start_conversion( void );

void ADC_HWI_FUNC( void );
void ADC_SWI_FUNC( UArg arg0, UArg arg1 );
void Process_Inputs(  UArg arg0, UArg arg1 );
void Update_LCD(  UArg arg0, UArg arg1 );

#endif /* APP_EE4930_LAB6_H_ */
