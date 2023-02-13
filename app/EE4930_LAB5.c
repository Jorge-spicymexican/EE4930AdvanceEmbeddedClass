/*
 * EE4930_LAB5.c
 *
 *  Created on: Jan 22, 2023
 *      Author: jurado-garciaj
 */

// Included files
#include <stdio.h>
#include "msp432.h"
#include "defines.h"
#include "msoe_lib_misc.h"

// Drivers
#include <MSP432P401R_GPIO.h>



void Init_Watchdog( void)
{
    //Configure WDT_A
    // use password must be written or the WDT resets
    // Using the BCLK clock source
    // Setting watchdog as interval timer mode
    //Clear count

    //for real run WDT_A_CTL_IS_2
    //demo WDT_A_CTL_IS_3
   WDT_A->CTL =  WDT_A_CTL_PW | WDT_A_CTL_SSEL__BCLK | WDT_A_CTL_TMSEL
               | WDT_A_CTL_CNTCL // clear the count
               | WDT_A_CTL_IS_3;
    return;
}


void init_CS( void )
{
    CS->KEY = CS_KEY_VAL;     //unlock
    CS->CTL1 = CS_CTL1_SELM__REFOCLK | CS_CTL1_SELS__REFOCLK
                | CS_CTL1_SELA__REFOCLK; // source REFOCLK for BCLK
    CS->KEY = 0;  //lock
    return;
}

//Power control module
/*
 * Allows for the optimization of power
 * This can work in conjunction with the Clock System
 * to control the power setting of the device and hence consumption
 * All peripherals and processors go to the PCM and then translate to the
 * Power Supply System and ClockSystem
 * Different Power Modes, Active MOde, Sleep Mode LPM0, LPM3 LPM4
 * For the lowest power consumption possible one must put the MCU in LPM3.5 or
 * LPM4.5 this only allows the board to have the WDT and RTC peripheral on.
 */
void Init_PCM( void )
{
    // unlock PCM/PMR
    // Entering LMPM3 when setting up the peripheral
    // Then when only activating the watchdog timer going into LPM4
    // Optional state is also LPM3.5 instead.
    //Watchdog is possible in LPM3.5
    // Operating state LDO_VCORE0 setting set to 0
    // Power Mode LPMR_LPM3
    // This is for Peripherals enabling
 PCM->CTL0 = PCM_CTL0_KEY_VAL | PCM_CTL0_LPMR__LPM3 |PCM_CTL0_AMR__AM_LF_VCORE0;
    return;
}


void Init_Temp( void )
{

    // Potentiometer for Room Temperature
    //A0
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN2);  //Set P5.2 as input for A3
    // input on A3 into MEM0
    P5->SEL0 |= BIT2; // use with A3
    P5->SEL1 |= BIT2; // use with A3

    //setup the ADCPeripheral
    // Setting up the control register
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL__MCLK
                                     | ADC14_CTL0_ON  | ADC14_CTL0_CONSEQ_0;

    // Sampling time, S&H=96, ADC14 on, SMCLK, with 14 Bit Resolutions
    // make sure read the reset operation first
    // CSTARTADDx this gets the start address, these bits select which
    //ADC14 conversion

    // memory registers is used for a single conversion or for the first
    //conversion in a sequence.
    // setting the memory to MEM0
    ADC14->CTL1 &= ~(ADC14_CTL1_RES_2 | ADC14_CTL1_RES_1);     // clear res bits
    ADC14->CTL1 |=  (ADC14_CTL1_RES_2|ADC14_CTL1_PWRMD_2);

    ADC14->CTL1 |= (3 << ADC14_CTL1_CSTARTADD_OFS); //Using MEM3


   //Memory conversion control 0 register being set to get data for A3
   ADC14->MCTL[3] |=  ADC14_MCTLN_INCH_3|ADC14_MCTLN_EOS;
   //EOS help to make sue the other channels do not get commuted

   // Enabled interrupt information for IER0 register
   ADC14->IER0 |= ( ADC14_IER0_IE3 );

   // enable the conversion
   ADC14->CTL0 |= ADC14_CTL0_ENC;

   //use pin
   // pin P4.5
   GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN5 );
   GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN5 );
}

