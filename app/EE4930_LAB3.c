/*
 * EE4930_LAB.c
 *
 *  Created on: Dec 31, 2022
 *      Author: jurado-garciaj
 */

#include "EE4930_LAB3.h"
#include "msp.h"
#include "stdio.h"
#include <MSP432P401R_GPIO.h>

void function_prototype_only( void)
{
    return;
}

void Small_Loops_rolling( void )
{
    int fact[5];
    fact[0] = 1;

    int i=0;
    // Overhead of managing a counter
    // just for 4 iterations
    // is not a good idea
    for ( i = 1; i < 5; ++i)
    {
        fact[i] = fact[i - 1] * i;
    }

    return;
}

void Calculation_loop( void )
{
    int arr[10000];
    int a = 1;
    int b = 5;
    int c = 2;
    uint32_t i;
    // Calculating a constant expression
    // for each iteration is not good.
    //Better way to pre-calculat the constant expression
    for (i = 0; i < 10000; ++i)
    {
        arr[i] = ( ((c % a) * a / b) % c) * i;

    }
    return;
}



//division and multiplication math
int div_multi_loop( void )
{
    uint32_t stuff = 0;
    while(0)
    {
        //do some stupid studd
        if( stuff < 1 )
        {
            stuff = 12321089;
            stuff = stuff << 100;
        }
        else
        {
            //for loop for nothing
            while(0);

        }
    }

    int a = 100;
    int b = 1;
    int c =23103-1;
    int d = a/ (b*a-c/2) %c;
    return 0;
}


//setup function for GPIO PIN to read speed data
void setup( void )
{
    //enable gpio pin to output and set to high
    GPIO_setAsOutputPin( GPIO_PORT_P2, GPIO_PIN3); // P2.3 as an output pin

    GPIO_setOutputLowOnPin( GPIO_PORT_P2, GPIO_PIN3 ); //Set P2.3 Low

    GPIO_setOutputHighOnPin( GPIO_PORT_P2, GPIO_PIN3 ); //Set P2.3 Low
}


void Complex_if_else ( void )
{
    uint32_t rand = 21321;

    if(rand < -1)
    {
        printf("hello world\n");
    }
    else if(rand < -1)
    {
        printf("hello world\n");
    }
    else if(rand < 40)
    {
        printf("hello world\n");
    }
    else if(rand < 100)
    {
        printf("hello world\n");
    }
    else if(rand < 1200)
    {
        printf("hello world\n");
    }
    else if(rand < 2100)
    {
        printf("hello world\n");
    }
    else if(rand < 2400)
    {
        printf("hello world\n");
    }
    else if(rand < 3500)
    {
        printf("hello world\n");
    }
    else if(rand < 3300)
    {
        printf("hello world\n");
    }
    else if(rand < 3500)
    {
        printf("hello world\n");
    }
    else if(rand < 3300)
    {
        printf("hello world\n");
    }
    else{printf("hello world\n");
    }
}

int never_used_function_1( void )
{
    //Screen char array that won't be used
    static const char start[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00,0x00, 0x00, 0x00, 0xe0, 0xf0,
                               0x48, 0x88,0x84, 0x84, 0x7c, 0x04,
                               0x84, 0x84, 0x8c,0x8c, 0x94, 0xb4,
                               0xe4, 0xa4, 0xa4, 0x28,0x48, 0x70,
                               0xe0, 0x00, 0x00, 0x00, 0x00,0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00,0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00,0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00,0x00, 0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00,0x00, 0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00,0x00,
                               0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00,0x00, 0x00, 0x00, 0x00, 0x00,
                               0x00, 0x00,0x00, 0xf8, 0x8e, 0xe1,
                               0xf8, 0xfc, 0x7c,0x44, 0x44, 0x44,
                               0x44, 0x45, 0x25, 0x25
                            };
}

int never_used_function_2( void )
{
    uint32_t i;
    P4->OUT^= 0b00111110;
    if(P4->OUT& 0b00111110)
    {
        ;
    }
    else
    {
        printf("hi\n");
    }

    for(i = 10000; i > 0; i--);

    return 0;
}

int never_used_function_3( void )
{
    uint32_t i;
    for(i = 10000; i > 0; i--)
    {
        printf("hello world");
    }
    never_used_function_3();
}

int never_used_function_4( void )
{
    never_used_function_3();
}
