/*
 * EE4930_FINAL_EXAM.c
 *
 *  Created on: Feb 14, 2023
 *      Author: jurado-garciaj
 */

#include "msp432.h"
#include "msoe_lib_all.h"
#include "EE4930_FINAL_EXAM.h"
#include "defines.h"

// Drivers
#include <MSP432P401R_GPIO.h>

SIgloo Igloo;
static float temp = 0;

//file scope functions
static void setup_inputs();
static void setup_outputs();
static eSystemState Temp_above_setpoint_Handler(void);
static eSystemState Temp_below_setpoint_Handler(void);


static sStateMachine sMachiney [LOOPUP_ROW][LOOPUP_COLUMN] =
{

 {
    //off state moves
    {Off_state,Temp_above_setpoint_event,Temp_above_setpoint_Handler},
    {Off_state,Temp_below_setpoint_event,Temp_below_setpoint_Handler},
 },
 {
    //on state moves
    {Active_state,Temp_above_setpoint_event,Temp_above_setpoint_Handler},
    {Active_state,Temp_below_setpoint_event,Temp_below_setpoint_Handler},
 },

};



void Igloo_init( void )
{
    Igloo.sInputs.Igloo_temp = 0;
    Igloo.sInputs.setpoint_temp = 0;
    Igloo.sOutputs.Fan_control = OFF;
    Igloo.sOutputs.Heating_Coil = OFF;
    Igloo.sOutputs.LCD_Display_flag = OFF;

    //init the gpio pins inputs
    setup_inputs();

    //init the gpio pins outputs
    setup_outputs();

    //MCU clock init
    //Clock_Init_48MHz();

    //Interrupt setup
    NVIC->ISER[0] |= ( 1 << 24 ); //NVIC for ADC14 at ISER[24]
    NVIC->ISER[1] |= ( 1 << 3  ); //NVIC for PORT 1 at ISER[35]
    NVIC->ISER[0] |= ( 1 << 25 ); //NVIC for TIMER_32_1 at ISER[25]

    __enable_interrupt();

    return;
}

//Read event
eSystemEvent Igloo_ReadEvent( void )
{
    eSystemEvent return_event = no_new_event;
    //check if the sensor readings are below or above setpoints
    if( (Igloo.sInputs.Igloo_temp ) > (Igloo.sInputs.setpoint_temp) )
    {
        //turn off
        return_event = Temp_above_setpoint_event;
    }

    if( (Igloo.sInputs.Igloo_temp ) < (Igloo.sInputs.setpoint_temp) )
    {
        //turn on
        return_event = Temp_below_setpoint_event;
    }

    //return no handler
    return return_event;
}

//polling of the Igloo LUK
eSystemState Igloo_Poll( eSystemEvent NewEvent, eSystemState NextState)
{
    //Check NULL pointer and array boundary
    if( ( NextState < last_state) && (NewEvent < Last_event) )
    {
        int i;
        //loop around the state machine until the state was found
        for( i=0; i < LOOPUP_COLUMN; i ++ )
        {
            //if the Event machines and Stata machine Event Handel also matches
            if( (sMachiney[NextState][i].eStateMachineEvent == NewEvent ) &&
                (sMachiney[NextState][i].pfStateMachineEvnentHandler != NULL ) )
            {
                //function call for the next new state
                // function call as per the state and event and return the next state of the finite state machine
                NextState = (*sMachiney[NextState][i].pfStateMachineEvnentHandler)();
            }
        } //end of for loops
    }

    return NextState;
}

void Igloo_UpdateLCD( void )
{
    if( Igloo.sOutputs.LCD_Display_flag)
    {
        //update the lcd
        //update the temp and setpoint
        LCD_goto_xy(7,1);
        LCD_print_str("    ");
        LCD_goto_xy(7,1);
        LCD_print_dec3(Igloo.sInputs.Igloo_temp);

        LCD_goto_xy(7,2);
        LCD_print_str("    ");
        LCD_goto_xy(7,2);
        LCD_print_dec3(Igloo.sInputs.setpoint_temp);

        Igloo.sOutputs.LCD_Display_flag = FALSE;

    }
    return;
}

//handler above set point
static eSystemState Temp_above_setpoint_Handler( void )
{
    //turn off fan
    //GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN6 );
    //TIMER_A2->CCR[0] = 0;
    Config_Duty(0);

    //turn off coil
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN2 );

    Igloo.sOutputs.Fan_control = OFF;
    Igloo.sOutputs.Heating_Coil = OFF;
    LCD_goto_xy(0,3);
    LCD_print_str("FAN: OFF");
    LCD_goto_xy(0,4);
    LCD_print_str("Coil: OFF");


    return Off_state;
}

//handler below set point
static eSystemState Temp_below_setpoint_Handler( void )
{
    //turn on coils
    GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN2 );

    //turn on fan
    GPIO_setOutputHighOnPin( GPIO_PORT_P6, GPIO_PIN6 );
    Config_Duty(20);

    Igloo.sOutputs.Fan_control = ON;
    Igloo.sOutputs.Heating_Coil = ON;
    LCD_goto_xy(0,3);
    LCD_print_str("FAN: ON ");
    LCD_goto_xy(0,4);
    LCD_print_str("Coil: ON ");

    return Active_state;
}

//Init the setup_inits
void setup_inputs( void )
{
   Stop_watchdog();    // stop Watch dog timer
   Set_ports_to_out();

   // For Humidity Setpoint - two pushbutton switches
   // SW1
   GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN1 );
   //Set P1.1 as an Input with Pull up resistor high
   P1->IES &= ~GPIO_PIN1; // enable edge select
   P1->IE |= GPIO_PIN1; // enable interrupt

   // SW2
   GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN4 );
   //Set P1.4 as an Input with Pull up resistor high
   P1->IES &= ~GPIO_PIN4; // enable edge select
   P1->IE |= GPIO_PIN4; // enable interrupt

   // Potentiometer for Igloo Temperature
   //A6
   // input on A3 into MEM1
   P5->SEL0 |= BIT2; // use with A3
   P5->SEL1 |= BIT2; // use with A3

   // Setting up the control register
   ADC14->CTL0 = ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP |
                ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_ON | ADC14_CTL0_CONSEQ_1;


   // Sampling time, S&H=96, ADC14 on, SMCLK, with 14 Bit Resolutions
   // make sure read the reset operation first
   // memory registers is used for a single conversion
   //or for the first conversion in a sequence.
   // setting the memory to MEM1
   ADC14->CTL1 &= ~ADC14_CTL1_RES_1;
   ADC14->CTL1 |= (1 << ADC14_CTL1_CSTARTADD_OFS) | ADC14_CTL1_RES_2;


   //Memory conversion control 0 register being set to get data for A3
   ADC14->MCTL[1] |= ( ADC14_MCTLN_INCH_3 | ADC14_MCTLN_EOS );

   // Enabled interrupt information for mem[1]
   ADC14->IER0 |= (1<<1);

   // enable the conversion
   ADC14->CTL0 |= ADC14_CTL0_ENC;

   //setting up of the timer32
   // Set master clock (MCLK) to HFXTCLK with divide by 1 - 48MHz
   TIMER32_1->LOAD |= 26875; //Value in which timer will get reloaded
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

   return;
}

//Init the setup_outputs
void setup_outputs( void )
{
    //Fan control Output control turn on 5V DC fan 6.6
    //setup of this gpio pin to a pwm signal of 17.5 Hz and 20% duty cycle
    //PWM PINS
    GPIO_setAsOutputPin( GPIO_PORT_P6, GPIO_PIN6); // P6.6 as an output pin
    GPIO_setOutputLowOnPin( GPIO_PORT_P6, GPIO_PIN6 );

    // input on A0 into MEM0
    P6->SEL0 |= GPIO_PIN6; // use with TA0.2
    P6->SEL1 &= ~GPIO_PIN6; // use with TA0.2
    // SET the TIMER in UP MODE WITH RESER/SET MODE
    // CLOCK SOURCE SMCLK 3MHZ
    // DIVIDE by 8
    // Mode Control: Up Mode
    // Interrupt for right now disabled
    TIMER_A2->CTL |= (TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_ID_3|TIMER_A_CTL_MC__UP);

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
    TIMER_A2->CCR[0] = 65534;
    //change this file in the Capture control register to change duty cycle
    //TIMER_A2->CCR[3] = 3250;

    //setup of the gpio pin of the heater coil 4.2
    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN2 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN2 );

    // Configures the LCD display screen clears display afterwards
    LCD_Config();

    //Printing "EE4930-011" on the first line of the LCD screen
    LCD_goto_xy(0,1);
    LCD_print_str("Temp F:");
    LCD_goto_xy(0,2);
    LCD_print_str("Setpt:");
    LCD_goto_xy(0,3);
    LCD_print_str("FAN:     ");
    LCD_goto_xy(0,4);
    LCD_print_str("Coil:    ");

    return;
}

//setting up of the duty cycle for pwm
int Config_Duty(float duty)
{
    //Duty 0-100
    //will need to grab current CCR[0] register value
    //and also grab the its current duty cycle also
    //current CCR[0] value:
    int current_val =  TIMER_A2->CCR[0];
    //which direction
    //pwmA which is P2.6 or PWMA
    float new_val = (float)( current_val*(duty/100) );
    TIMER_A2->CCR[3] = (int) new_val;

    return  (float)(100*new_val)/current_val;
}

// Interrupt Handler for SW1
void PORT1_IRQHandler()
{
    int readIV = P1->IV; // Reading IV register to clear

    switch( readIV )
    {
    case 0x04: // P1.1
        Igloo.sInputs.setpoint_temp += 1;
        Igloo.sOutputs.LCD_Display_flag = TRUE;
        break;
    case 0x0A: // P1.4
        Igloo.sInputs.setpoint_temp -= 1;
        Igloo.sOutputs.LCD_Display_flag = TRUE;
        break;
    }

    if(Igloo.sInputs.setpoint_temp > 250)
    {
        Igloo.sInputs.setpoint_temp = -250;
    }

    if( Igloo.sInputs.setpoint_temp < -250 )
    {
        Igloo.sInputs.setpoint_temp = 250;
    }

}

// Interrupt Handler for TIMER32
void T32_INT1_IRQHandler()
{

    TIMER32_1->INTCLR = CLEAR;
    // start a new A/D conversion
    ADC14->CTL0 |= ADC14_CTL0_SC;
}

//Interrupt Handler for ADC14
void ADC14_IRQHandler(void)
{
    static int previous_adc = 0;
    int readIV = ADC14->IV; // Reading the IV register to clear

    int adc_reading = ADC14->MEM[1];
    float adc_voltage = ( ( (float)(adc_reading)/4096 ) * (3300.0f));
    int temp_c = adc_voltage/10 - 50; // y = x10mV/C - 75
    temp = ( temp_c * (9.0f / 5.0f) ) + 32.0;

    Igloo.sInputs.Igloo_temp = (int)(temp);

    //check if the value is different
    if( previous_adc != Igloo.sInputs.Igloo_temp )
    {
        Igloo.sOutputs.LCD_Display_flag = TRUE;
    }

    previous_adc = Igloo.sInputs.Igloo_temp;
}
