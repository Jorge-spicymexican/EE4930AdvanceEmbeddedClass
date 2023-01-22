/*
 * EE4930_LAB4.c
 *
 *  Created on: Jan 2, 2023
 *      Author: jurado-garciaj
 */

// Included files
#include <stdio.h>
#include "msp432.h"
#include "msoe_lib_all.h"
#include "defines.h"
#include "EE4930_LAB4.h"

// Drivers
#include <MSP432P401R_GPIO.h>


sDehumidifier Honey;

//private function prototypes
static void Initlab4_A2D( void );
static void setup_outputs( void );
static void setup_inputs( void );
//function call to dispatch the amount and return the ideal state
static eSystemState Humidity_above_setpoint_Ice_off_Handler(void);
static eSystemState Humidity_below_setpoint_Ice_off_Handler(void);
static eSystemState Humidity_within_setpoint_Ice_off_Handler(void);
static eSystemState Ice_Sensed_Handler(void);
static eSystemState Ice_Not_Senses_Handler(void);


//public function prototypes
void Dehumidifier_Read( eSystemInputs inputs );
eSystemEvent Dehumidifier_ReadEvent( void );
eSystemState Dehumidifier_Poll( eSystemEvent NewEvent, eSystemState NextState);
void Dehumidifier_init( void );

//module specific structure
//Initialize array of structure with states and event with proper handler
sStateMachine sMachiney [LOOPUP_ROW][LOOPUP_COLUMN] =
{

 {
    //off state moves
    {Off_state,Ice_sensed_event,Ice_Sensed_Handler},
    {Off_state,Humidity_above_setpoint_Ice_off_event,Humidity_above_setpoint_Ice_off_Handler},
    {Off_state,Humidity_within_setpoint_Ice_off_event,Humidity_within_setpoint_Ice_off_Handler},
    {Off_state, Humidity_below_setpoint_Ice_off_event, Humidity_below_setpoint_Ice_off_Handler}

 },
 {
 //Defrost state moves
   {Defrost_state,Ice_off_event,Ice_Not_Senses_Handler},
   {Defrost_state,Ice_sensed_event,Ice_Sensed_Handler},
   {Defrost_state,Humidity_below_setpoint_Ice_off_event,Humidity_below_setpoint_Ice_off_Handler},
   {Defrost_state, Humidity_above_setpoint_Ice_off_event, Humidity_above_setpoint_Ice_off_Handler}

},
 {
    //active state moves
    {Active_state,Ice_sensed_event,Ice_Sensed_Handler},
    {Active_state,Humidity_below_setpoint_Ice_off_event,Humidity_below_setpoint_Ice_off_Handler},
    {Active_state,Humidity_within_setpoint_Ice_off_event,Humidity_within_setpoint_Ice_off_Handler},
    {Active_state, Humidity_above_setpoint_Ice_off_event, Humidity_above_setpoint_Ice_off_Handler}
 },

};


//state machine info
//function call to dispatch the amount and return the ideal state
eSystemState Humidity_above_setpoint_Ice_off_Handler(void)
{
    //run outputs
    //turn on fan
    GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN1 ); //fan

    //Turn the compressor on
    // make it a green led
    GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN1 ); //Green
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 ); //Red

    //turn the defrost mode off
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN2 ); //Green

    LCD_goto_xy(0,1);
    LCD_print_str("FAN/COMP   ");
    LCD_goto_xy(0,2);
    LCD_print_str("ON/ON      ");

    Honey.sOutputs.Fan_control = ON;
    Honey.sOutputs.Compressor = ON;
    Honey.sOutputs.LCD_Display_flag = TRUE;
    //Honey.eCurrentState = Active_state;

    return Active_state;
}


eSystemState Humidity_below_setpoint_Ice_off_Handler(void)
{
    //run outputs
    //turn off fan
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 ); //fan

    //Turn the compressor off
    // make it a red led
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN1 ); //Green
    GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN0 ); //Red

    //turn the defrost mode off
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN2 ); //Green

    //show on LCD we are in defrost mode
    LCD_goto_xy(0,1);
    LCD_print_str("FAN/COMP   ");
    LCD_goto_xy(0,2);
    LCD_print_str("OFF/OFF    ");

    Honey.sOutputs.Fan_control = OFF;
    Honey.sOutputs.Compressor = OFF;
    Honey.sOutputs.LCD_Display_flag = TRUE;
    //Honey.eCurrentState = Off_state;

    return Off_state;
}


eSystemState Humidity_within_setpoint_Ice_off_Handler(void)
{
    //run outputs and check if this true
    // keep the same outputs as from before
    // just change the LCD update to true still
    //turn off the defrost mode led
    GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2 ); //Deforst mode pin off
    Honey.sOutputs.LCD_Display_flag = TRUE;
    Honey.eCurrentState = Honey.eCurrentState;

    //check the current state value
    if( Honey.eCurrentState == Active_state)
    {
        //run the active state output
        Humidity_above_setpoint_Ice_off_Handler();
    }
    if( Honey.eCurrentState == Off_state)
    {
        //run the last state outputs
        Humidity_below_setpoint_Ice_off_Handler();
    }

    return Honey.eCurrentState;
}


eSystemState Ice_Sensed_Handler(void)
{
    //run outputs and check if this true
    //if Ice sensed turn on the fan
    GPIO_setOutputHighOnPin( GPIO_PORT_P4, GPIO_PIN1 );

    //Turn the compressor off
    // make it a red led
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN1 ); //Green
    GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN0 ); //Red

    //show on LCD we are in defrost mode
    GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2 ); //Blue

    LCD_goto_xy(0,1);
    LCD_print_str("FAN/COMP  ");
    LCD_goto_xy(0,2);
    LCD_print_str("ON/OFF    ");

    Honey.sOutputs.Fan_control = ON;
    Honey.sOutputs.Compressor = OFF;
    Honey.sOutputs.LCD_Display_flag = TRUE;
   // Honey.eCurrentState = Defrost_state;

    return Defrost_state;
}


eSystemState Ice_Not_Senses_Handler(void)
{
    // run outputs since we are not sensing ice no more and move to its next state

    //show on LCD we not in defrost mode
    //BLUE
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN2);

    Honey.sOutputs.LCD_Display_flag = TRUE;

    //need to figure out how to fill in this previous state value
    // this can be done by configuring the inputs
    if( Honey.sInputs.Humidity_percentage > (Honey.sInputs.Humidity_setpoints + 5) )
    {
        //humidity percentage is greater than set points return to dehumidification process
        //this means we are in active state and should run outputs for this mode
        return Humidity_above_setpoint_Ice_off_Handler();

    }
    if( Honey.sInputs.Humidity_percentage < (Honey.sInputs.Humidity_setpoints - 5) )
       {
           //humidity percentage is greater than set points return to dehumidification process
           //this means we are in active state and should run outputs for this mode
           return Humidity_below_setpoint_Ice_off_Handler();

       }

    //if its within the mid band this means
    //run the previous state output and check for the value
    return Honey.ePreviousState;
}


eSystemState Dehumidifier_Poll( eSystemEvent NewEvent, eSystemState NextState)
{
    static eSystemState State = last_state;

    //Check NULL pointer and array boundary
    if( ( NextState < last_state) && (NewEvent < Last_event) )
    {

        int i = 0;

        //loop around the state machine until the state was found
        for(i=0; i < LOOPUP_COLUMN; i ++ )
        {
            //if the Event machines and Stata machine Event Handel also matches
            if( (sMachiney[NextState][i].eStateMachineEvent == NewEvent ) &&
                (sMachiney[NextState][i].pfStateMachineEvnentHandler != NULL ) )
            {
                State = NextState;
                // function call as per the state and event and return the next state of the finite state machine
                NextState = (*sMachiney[NextState][i].pfStateMachineEvnentHandler)();
                Honey.Errorcode = FALSE;

                //only update previous state if we are not in defrost mode. We will need this state back
                if( NextState != Defrost_state)
                {
                    //save the previous state;
                    Honey.ePreviousState = State;
                }
                Honey.eCurrentState = NextState;
            }
        }

    }
    else
    {
        //Invalid
        Honey.Errorcode = TRUE;
    }

    return NextState;
}


//Read the event
eSystemEvent Dehumidifier_ReadEvent( void )
{

    //check if we are in the defrost state
    if( Honey.eCurrentState == Defrost_state )
    {
        //if sensor is false go to the previous state in which it was in
        if( Honey.sInputs.Ice_sensor == FALSE)
        {
            return Ice_off_event;
        }

        return Ice_sensed_event;
    }

    //if we are not in the defrost current state mode
    //Based on the inputs check which even to trigger
    if( Honey.sInputs.Ice_sensor == TRUE)
    {
        return Ice_sensed_event;
    }
    else
    {
        //check if the Sensor readings
        if( Honey.sInputs.Humidity_percentage > (Honey.sInputs.Humidity_setpoints + 5))
        {
            //humidity percentage is greater than set points
            return Humidity_above_setpoint_Ice_off_event;

        }

        if( Honey.sInputs.Humidity_percentage < (Honey.sInputs.Humidity_setpoints - 5))
        {
            //humidity percentage is greater than set points
            return Humidity_below_setpoint_Ice_off_event;

        }

        //if neither of these statements are true then we are in idle mode
        return Humidity_within_setpoint_Ice_off_event;

    }

}


//Read the global variable from
void Dehumidifier_Read( eSystemInputs inputs )
{
    Honey.sInputs.Humidity_percentage = inputs.Humidity_percentage;
    Honey.sInputs.Humidity_setpoints = inputs.Humidity_setpoints;
    Honey.sInputs.Ice_sensor = inputs.Ice_sensor;
    Honey.sInputs.Room_temperature = inputs.Room_temperature;

    //update LCD Humidity percentages, set-points, and room temperature
    LCD_goto_xy(8,3);
    LCD_print_udec3(Honey.sInputs.Room_temperature);
    LCD_goto_xy(8,4);
    LCD_print_udec3(Honey.sInputs.Humidity_setpoints);
    LCD_goto_xy(8,5);
    LCD_print_udec3(Honey.sInputs.Humidity_percentage);

}


// Init the dehumidifer
void Dehumidifier_init( void )
{
    Honey.eCurrentState = Off_state;
    Honey.ePreviousState = Off_state;
    Honey.sInputs.Humidity_percentage = 0;
    Honey.sInputs.Humidity_setpoints = 0;
    Honey.sInputs.Ice_sensor = FALSE;
    Honey.sInputs.Room_temperature = 0;
    Honey.sOutputs.Compressor = OFF;
    Honey.sOutputs.Fan_control = OFF;
    Honey.sOutputs.LCD_Display_flag = OFF;

    setup_inputs();

    setup_outputs();

    return;
}


//Init the setup_outputs
void setup_outputs( void )
{
    //Fan control Output control turn on 5V DC fan 4.1
    GPIO_setAsOutputPin( GPIO_PORT_P4, GPIO_PIN1 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P4, GPIO_PIN1 );
    P4->DS |= BIT1;


    // Compressor output by using MSP on board led
    //Red
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN0 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN0 );

    //Green
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN1 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN1 );

    //BLUE
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN2 );
    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN2);


    // Configures the LCD display screen clears display afterwards
    LCD_Config();

    //Printing "EE4930-011" on the first line of the LCD screen
    LCD_goto_xy(0,1);
    LCD_print_str("FAN/COMP");
    LCD_goto_xy(0,2);
    LCD_print_str("OFF     ");

    LCD_goto_xy(0,3);
    LCD_print_str("Temp (F):");
    LCD_goto_xy(0,4);
    LCD_print_str("Setpoint:");
    LCD_goto_xy(0,5);
    LCD_print_str("Humidity:");


    return;

}


//Init the setup_inits
void setup_inputs( void )
{

   Stop_watchdog();    // stop Watch dog timer

   Set_ports_to_out(); //sets all the ports to outputs to prevent floating inputs

   // For Humidity Setpoint - two pushbutton switches
   // SW1
   GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN1 ); //Set P1.1 as an Input with Pull up resistor high
   P1->IES &= ~GPIO_PIN1; // enable edge select
   P1->IE |= GPIO_PIN1; // enable interrupt

   // SW2
   GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN4 ); //Set P1.4 as an Input with Pull up resistor high
   P1->IES &= ~GPIO_PIN4; // enable edge select
   P1->IE |= GPIO_PIN4; // enable interrupt

   // Potentiometer for Room Temperature
   //A0
   GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN5);  //Set P5.5 as input for A0
   // input on A0 into MEM0
   P5->SEL0 |= BIT5; // use with A0
   P5->SEL1 |= BIT5; // use with A0

   // Potentiometer for Humidity
   //A2
   GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN3);  //Set P5.3 as input for A2
   // input on A2 into MEM2
   P5->SEL0 |= BIT3; // use with A2
   P5->SEL1 |= BIT3; // use with A2

   // Ice Senior - yes.no (switch or jumper) //setup pin P5.4 as input
   GPIO_setAsInputPin(GPIO_PORT_P5, GPIO_PIN4);  //Set P5.4 as input ice stuff


   Initlab4_A2D();


   //init Timer32 for ice sensor
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

   return;

}

//init for A2D converter
void Initlab4_A2D( void )
{

    // Setting up the control register
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL__SMCLK | ADC14_CTL0_ON | ADC14_CTL0_CONSEQ_1;

    // Sampling time, S&H=96, ADC14 on, SMCLK, with 14 Bit Resolutions
    // make sure read the reset operation first
    // CSTARTADDx this gets the start address, these bits select which ADC14 conversion
    // memory registers is used for a single conversion or for the first conversion in a sequence.
    // setting the memory to MEM0
    ADC14->CTL1 &= ~ADC14_CTL1_RES_3;
    ADC14->CTL1 |= ADC14_CTL1_RES_3;
    ADC14->CTL1 |= (0 << ADC14_CTL1_CSTARTADD_OFS); //A0
    //ADC14->CTL1 |= (2 << ADC14_CTL1_CSTARTADD_OFS); //A2

   //Memory conversion control 0 register being set to get data for A0
   ADC14->MCTL[0] |=  ADC14_MCTLN_INCH_0;

   //Memory conversion control 2 register being set to get data for A2
   ADC14->MCTL[2] |=  ( ADC14_MCTLN_INCH_2 | ADC14_MCTLN_EOS) ;

   // Enabled interrupt information for IER0 register
   ADC14->IER0 |= (ADC14_IER0_IE0 | ADC14_IER0_IE2 );

   // enable the conversion
   ADC14->CTL0 |= ADC14_CTL0_ENC;

   return;
}


