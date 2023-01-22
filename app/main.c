/* Copyright (C) 2022 MSOE
 *
 * All Rights Reserved
 * You may not use, distribute or modify this code without the
 * express written permission of MSOE
 *
 * Contact Info
 * jorgejuradogarcia2@gmail.com
 * 608-312-5950
 *
 */

#include <stdio.h>
#include "msp432.h"
#include "msoe_lib_all.h"
#include "defines.h"
#include <MSP432P401R_GPIO.h>

//Preprocessors
#define LAB1_ON FALSE
#define LAB2_ON FALSE
#define LAB3_ON FALSE
#define LAB4_ON TRUE
#define CLEAR 21


// Lab modules
#if LAB1_ON
#include "EE4930_LAB1.h"
#endif

#if LAB2_ON
#include "EE4930_LAB2.h"
#endif

#if LAB3_ON
#include "EE4930_LAB3.h"
#endif


#if LAB4_ON
#include "EE4930_LAB4.h"
#endif

/*
 * main.c
 */

//global variables for Lab 2
unsigned adc_val;
float ADC_Percentage;
float PWM_dutycycle;
int Calculated_dutycycle;


//global variables for Lab4
unsigned adc_val_A0;
unsigned adc_val_A2;
float ADC_A2_Percentage;
float ADC_A0_Percentage;
unsigned char ticks;

eSystemInputs Inputs;

int main(void){

#if LAB1_ON
     Init_Lab1();

    while(1)
    {
        Lab1_Poll();
    }
#endif

#if LAB2_ON
    Clock_Init_48MHz();
    Init_Lab2();

    NVIC->ISER[0] |= ( 1<<24 ); //NVIC for ADC14 at ISER[24]
    NVIC->ISER[1] |= ( 1<<3 ); //NVIC for PORT 1 at ISER[35]
    NVIC->ISER[0] |= ( 1<<25 ); //NVIC for TIMER_32_1 at ISER[25]
    __enable_interrupt();

    while(1)
    {
        if(LCD_SET_FLAG == TRUE)
        {

            LCD_goto_xy(5, 3); // start at row 3, column 0
            LCD_print_udec5(adc_val);
            LCD_goto_xy(7, 4); // start at row 4, column 0
            LCD_print_udec5((int)PWM_dutycycle);
            LCD_SET_FLAG = FALSE;
        }
    }
#endif

#if LAB3_ON

    uint32_t counter = 0;
    setup();

    Small_Loops_rolling();

    Calculation_loop();

    div_multi_loop();

    Complex_if_else();

    function_prototype_only();

    if( counter > 0)
    {
        never_used_function_1();
        never_used_function_2();
        never_used_function_3();
        never_used_function_4();
    }

    //set P2.3 to low
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN3 ); //Set P2.3 Low
#endif

#if LAB4_ON
    Clock_Init_48MHz();

    //Interrupt setup
    NVIC->ISER[0] |= ( 1<<24 ); //NVIC for ADC14 at ISER[24]
    NVIC->ISER[1] |= ( 1 << 3 ); //NVIC for PORT 1 at ISER[35]
    NVIC->ISER[0] |= ( 1<<25 ); //NVIC for TIMER_32_1 at ISER[25]

    __enable_interrupt();
    Dehumidifier_init();

    eSystemEvent eNewEvent = Last_event;
    eSystemState eNextState  = Off_state;

    while(1)
    {
        if( ticks == TICKS_MAX)
        {
            //read the inputs
            Dehumidifier_Read(Inputs);

            //Run the state machine
            eNewEvent = Dehumidifier_ReadEvent();


            //poll of the state machine
            eNextState = Dehumidifier_Poll(eNewEvent, eNextState );
        }

    }

#endif

} // end main

#if LAB2_ON
// Interrupt Handler for ADC14
void ADC14_IRQHandler()
{
    int readIV = ADC14->IV; // Reading the IV register to clear
    // read A/D result
    adc_val = ADC14->MEM[0];
    ADC_Percentage = 100 * (float)adc_val/1024;
    PWM_dutycycle = 100 - ADC_Percentage;
    //printf("A/D reading = %d, ADC % = %f \n", adc_val, ADC_Percentage);
    //printf("Duty Cycle = %f\n", PWM_dutycycle);
    Calculated_dutycycle = Set_Duty( PWM_dutycycle );

}

// Interrupt Handler for SW1
void PORT1_IRQHandler()
{
    int readIV = P1->IV; // Reading IV register to clear
    // and set a LCD_flag to start to update with new values
    LCD_SET_FLAG = TRUE;

}

//Interrupt Handler for TIMRE32_Int1
void T32_INT1_IRQHandler()
{

    TIMER32_1->INTCLR = CLEAR;
    // start a new A/D conversion
    ADC14->CTL0 |= ADC14_CTL0_SC;
    //printf("starting ADC Timer\n");
}
#endif

#if LAB4_ON

// Interrupt Handler for ADC14
// Will enter into this interrupt source with the Interrupt of the ADC14 was set
void ADC14_IRQHandler()
{
    int readIV = ADC14->IV; // Reading the IV register to clear
    // read A/D result
    switch( readIV )
    {
        case 0x0C:
            adc_val_A0 = ADC14->MEM[0];
            ADC_A0_Percentage = 100 * (float)adc_val_A0/16384;
            //reading humidiity
            Inputs.Humidity_percentage = ADC_A0_Percentage;
            break;
        case 0x10:
            adc_val_A2 = ADC14->MEM[2];
            ADC_A2_Percentage =   (float)adc_val_A2/16384;
            //READING Room Temperature
            // 40-90 degrees so the values can range from 50 points
            Inputs.Room_temperature = (int)(ADC_A2_Percentage*50) + 40;
            break;

    }

}

// Interrupt Handler for SW1
void PORT1_IRQHandler()
{
    int readIV = P1->IV; // Reading IV register to clear

    switch( readIV )
    {
    case 0x04: // P1.1
        Inputs.Humidity_setpoints += 5;
        break;
    case 0x0A: // P1.4
        Inputs.Humidity_setpoints -= 5;
        break;
    }

    if(Inputs.Humidity_setpoints > 100)
    {
        Inputs.Humidity_setpoints = 100;
    }

}

//Interrupt Handler for TIMRE32_Int1
void T32_INT1_IRQHandler()
{

    TIMER32_1->INTCLR = CLEAR;
    // Read the Ice sensor pin
    Inputs.Ice_sensor =   ( (P5->IN & GPIO_PIN4) >> 4 );
    // run adc converter
    ADC14->CTL0 |= ADC14_CTL0_SC;

    ticks++;
    if( ticks > TICKS_MAX)
    {
        ticks = 0;
    }

}


#endif
