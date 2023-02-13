/*
 * EE4930_LAB6.c
 *
 *  Created on: Feb 2, 2023
 *      Author: jurado-garciaj
 *
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
 */

#include "EE4930_LAB6.h"
#include "msp.h"

#include <stdio.h>
#include <string.h>

#include "msoe_lib_clk.h"
#include "msoe_lib_lcd.h"
#include "msoe_lib_delay.h"
#include <MSP432P401R_GPIO.h>


void lcd_init( void )
{
    LCD_Config();
    LCD_clear();
    LCD_home();
    LCD_contrast(10);
    LCD_goto_xy(0 , 2);
    LCD_print_str("Temp: xx F");
}


void adc_init( void )
{
    //A0
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN5);  //Set P5.5 as input for A0
    // input on A0 into MEM0
    P5->SEL0 |= BIT5; // use with A0
    P5->SEL1 |= BIT5; // use with A0

    // Setting up the control register
     ADC14->CTL0 |= ADC14_CTL0_SHT0_5      | ADC14_CTL0_SHP |
                    ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_ON  |
                    ADC14_CTL0_CONSEQ_0;

     // Sampling time, S&H=96, ADC14 on, SMCLK, with 10 Bit Resolutions
     // make sure read the reset operation first
     // CSTARTADDx this gets the start address, these bits select which
     //ADC14 conversion
     // memory registers is used for a single conversion or for the
     //first conversion in a sequence.
     // setting the memory to MEM0
     ADC14->CTL1 &= ~ADC14_CTL1_RES_3;
     ADC14->CTL1 |= ADC14_CTL1_RES_2;
     ADC14->CTL1 |= (0 << ADC14_CTL1_CSTARTADD_OFS); //A0

     //Memory conversion control 0 register being set to get data for A0
     ADC14->MCTL[0] |=  ADC14_MCTLN_INCH_0;

     // Enabled interrupt information for IER0 register
     ADC14->IER0 |= ADC14_IER0_IE0;

     // enable the conversion
     ADC14->CTL0 |= ADC14_CTL0_ENC;

}

void start_conversion( void )
{
    ADC14->CTL0 |= 1; // start ADC conversions
}



