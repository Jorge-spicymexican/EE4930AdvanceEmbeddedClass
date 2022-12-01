/*
 * EE4930_LAB1.c
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

// Included files
#include <stdio.h>
#include "msp432.h"
#include "msoe_lib_all.h"
#include "defines.h"

// Drivers
#include <MSP432P401R_GPIO.h>

void PushButton_Pressed( void );
void PushButton_NotPressed( void );

void Init_Lab1( void )
{
    Stop_watchdog();    // stop Watch dog timer

    Set_ports_to_out(); //sets all the ports to outputs to prevent floating inputs
    LCD_Config(); // Configures the LCD display screen clears display afterwards

    GPIO_setAsInputPinWithPullUpResistor( GPIO_PORT_P1, GPIO_PIN1 ); //Set P1.1 as an Input with Pull up resistor high

    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN2); // P2.2 as an output pin

    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN2 ); //Set P2.2 Low

    //Printing "EE4930-011" on the first line of the LCD screen
    LCD_goto_xy(0,1);
    LCD_print_str("EE4930-011");



}


void Lab1_Poll( void )
{
    static char Previous_Press = 0;
    char Current_Press;

    // read the press from the input
    Current_Press = GPIO_getInputPinValue( GPIO_PORT_P1, GPIO_PIN1 );

    if( Previous_Press != Current_Press)
    {
        if( Current_Press == PRESSED ) // value will be 0 if pressed
        {
            PushButton_Pressed();
        }
        else
        {
            PushButton_NotPressed();
        }
    }

    Previous_Press = Current_Press;

    // do a while loop, while pushbutton
}

void PushButton_Pressed( void )
{
    //turn on an LED
    GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN2 ); //Set P2.2 Low

    // Display text on the second and third row
    LCD_goto_xy(0,2);
    LCD_print_str("OFF");

    LCD_goto_xy(0,3);
    LCD_print_str("Jorge");

    LCD_goto_xy(0,4);
    //blank out the Fourth row
    LCD_print_str("            "); //clearing by writing nothing

}

void PushButton_NotPressed( void )
{
   //Turn off an LED
   GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN2 ); //Set P2.2 Low

   // Display text on the second and fourth row
   LCD_goto_xy(0,2);
   LCD_print_str("ON ");

   LCD_goto_xy(0,4);
   LCD_print_str("Jurado");

   LCD_goto_xy(0,3);

   //blank out the third row
   LCD_print_str("            "); //clearing by writing nothing

}
