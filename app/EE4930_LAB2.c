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
 *    P5.5        IN      NONE            A0
 *    P6.6        OUT      N/A            Logic Pro/TA23
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
 *  We will be using TIMER_32 to START ADC CONVERTER after the timer expires and reaches zero
 *  My idea is to generate start the ADC converter every second this will have to be based on the CPU CLock
 *  Reasoning on why to use TIMER_32 in order to start the ADC converter is for its simplicity and easy to use.
 *  There is no need for to make it make for complex then it needs to be
 *  48 MHZ per tick therefore for one second we will need
 *      1/ (48 MHZ/256) Times per tick
 *      on a 32 bit Timer time after overflow is: 0.25 seconds almost 0.5 second
 *
 *
 *
NOTE:
    An interrupt is generated when the full 32-bit counter reaches zero and is only cleared when the
    T32INTCLRx register is written to. A register holds the value until the interrupt is cleared. The most
    significant carry bit of the counter detects the counter reaching zero.
    The interrupts can be masked by writing 0 to the Interrupt Enable bit in the T32CONTROLx register. Both
    the raw interrupt status, prior to masking, and the final interrupt status, after masking, can be read from
    status registers. The interrupts from the individual counters, after masking, are logically ORed into a
    combined interrupt, TIMINTC, provides an additional interrupt condition from the Timer32 peripheral. Thus,
    the module supports three interrupts in total – TIMINT1, TIMINT2, and TIMINTC.
 *
 *
 *
 * Set master clock (MCLK) to HFXTCLK with divide by 1 - 48MHz
 * Set high speed sub-system clock (HSMCLK) to HFXTCLK with divide by 2 - 24MHz (max allowed)
 * Set low speed sub-system clock (SMCLK) to HFXTCLK with divide by 4 - 12MHz (max allowed)
 * Set the auxiliary clock (ACLK) to REFOCLK - 32KHz
 * Set the backup clock (BCLK) to REFOCLK - 32KHz
 *
 */
// Included files
#include <stdio.h>
#include "msp432.h"
#include "msoe_lib_all.h"
#include "defines.h"

// Drivers
#include <MSP432P401R_GPIO.h>

void Init__Timer32( void );

void Init_A2D( void );

void Init_TimerA( void );

int Set_Duty(float duty);



void Init_Lab2( void )
{
    Stop_watchdog();    // stop Watch dog timer

    Set_ports_to_out(); //sets all the ports to outputs to prevent floating inputs
    LCD_Config(); // Configures the LCD display screen clears display afterwards

    // SW1
    GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN1 ); //Set P1.1 as an Input with Pull up resistor high
    P1->IES &= ~GPIO_PIN1; // enable edge select
    P1->IE |= GPIO_PIN1; // enable interrupt


    //PWM PINS
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN6); // P6.6 as an output pin
    // input on A0 into MEM0
    P6->SEL0 |= GPIO_PIN6; // use with TA0.2
    P6->SEL1 &= ~GPIO_PIN6; // use with TA0.2

    //A0
    GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN5);  //Set P5.5 as input for A0
    // input on A0 into MEM0
    P5->SEL0 |= BIT5; // use with A0
    P5->SEL1 |= BIT5; // use with A0


    //Printing "EE4930-011" on the first line of the LCD screen
    LCD_goto_xy(0,1);
    LCD_print_str("EE4930-011");
    LCD_goto_xy(0,2);
    LCD_print_str("Lab 2 ");
    LCD_goto_xy(0,3);
    LCD_print_str("ADC:");
    LCD_goto_xy(0,4);
    LCD_print_str("DUTY %:");

    Init_A2D();

    Init_TimerA();

    Init__Timer32();

}

void Init_TimerA( void )
{
    // SET the TIMER in UP MODE WITH RESER/SET MODE
    // CLOCK SOURCE SMCLK 3MHZ
    // DIVIDE by 8
    // Mode Control: Up Mode
    // Interrupt for right now disabled
    TIMER_A2->CTL |= (TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_ID_0|TIMER_A_CTL_MC__UP);

    //TA0.2
    // Set output mode to Reset and SET
    TIMER_A2->CCTL[3] |= (TIMER_A_CCTLN_OUTMOD_7);

    // Capture Control register to TIMER_Ax Capture.Compare
    //12 MHZ /3 = 4,000,000 input frequency
    // T_timer = 1/f_systemclock = 1/4 MHz = 0.25 uS
    // CTL Frequency = 100K HZ frequency
    // If using the full 16-bit timer we have 65535 ticks
    // For maximum speed before tick overflow we will have
    // t_overflow = tick_max * T_timer = 65535 * 0.25 us = 16.38375 mS
    //This is fast be good enough for our info
    TIMER_A2->CCR[0] = 6500;

    //change this file in the Capture control register to change duty cycle
    TIMER_A2->CCR[3] = 3250;

}

void Init__Timer32( void )
{
    // Set master clock (MCLK) to HFXTCLK with divide by 1 - 48MHz
    TIMER32_1->LOAD |= 46875; //Value in which timer will get reloaded
    /*
     *  TIMER CLOCK SPEED: 48MHz MHz
     *  TImer disabled
     *  Periodic mode Enabled
     *  Timer Interrupt enabled
     *  Clock division of 256
     *  Clock After Division in 48MHz/256 = 0.1875 MHz = 187,500 Hz
     *  Size of 16-bit counter
     *  Ticks 16 bits = 65535 ticks
     *  Timer_sec = 1/187500 = 5.333*10^(-6)
     *  Timer After expiration = 5.333*10^(-6) * 46875 = 0.25 seconds
     *  Wrapping mode
     *  32-bit counter
     */

    /*
     *
     */
    TIMER32_1->CONTROL |= 0x6A;

    // ENABLE TIMER
    TIMER32_1->CONTROL |= 0x80;

}

void Init_A2D( void )
{

   // Setting up the control register
   ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_ON | ADC14_CTL0_CONSEQ_0;

   // Sampling time, S&H=96, ADC14 on, SMCLK, with 10 Bit Resolutions
   // make sure read the reset operation first
   // CSTARTADDx this gets the start address, these bits select which ADC14 conversion
   // memory registers is used for a single conversion or for the first conversion in a sequence.
   // setting the memory to MEM0
   ADC14->CTL1 &= ~ADC14_CTL1_RES_2;
   ADC14->CTL1 |= ADC14_CTL1_RES_2;
   ADC14->CTL1 |= (0 << ADC14_CTL1_CSTARTADD_OFS); //A0

   //Memory conversion control 0 register being set to get data for A0
   ADC14->MCTL[0] |=  ADC14_MCTLN_INCH_0;

   // Enabled interrupt information for IER0 register
   ADC14->IER0 |= ADC14_IER0_IE0;

   // Interrupt enable 1 register information with overflow-interrupt enabled
   //ADC14->IER1 |= ADC14_IER1_OVIE;

   // Interrupt Vector Register for MEM0 0Ch
   ADC14->IV |= 0x0C;

   // enable the conversion
   ADC14->CTL0 |= ADC14_CTL0_ENC;

}


int Set_Duty(float duty){
    //Duty 0-100
    //will need to grab current CCR[0] register value
    //and also grab the its current duty cycle also
    //current CCR[0] value:
    int current_val =  TIMER_A2->CCR[0];
    //which direction
    //pwmA which is P2.6 or PWMA
    float new_val = (float)( current_val*(duty/100) );
    printf("Inserted CCR3 value for timer %d\n", (int)new_val);
    TIMER_A2->CCR[3] = (int) new_val;

    return  (float)(100*new_val)/current_val;
}


