//*****************************************************************************
//
// EE493_exam1_p2.c
// 1/20/2023
//
// Optimize by hand the following code for speed.
// Reference the line numbers of the code you change in each section of your
// solution
// Include comments to indicate what you did and how it improves execution speed
//
//****************************************************************************
#include "msp.h"

//int power(int x, int y);

//change the function from non-inline to  inline
//this will help to eliminate call-linkage overhead and expose sign optimization
static inline void init_A2D(void)
{
    // Sampling time, S&H=96, ADC14 on, ACLK

    //changed the modifications of the register to just one isstance.
    //ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_3;
    //ADC14->CTL0 |= ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_ON | ADC14_CTL0_MSC;
    ADC14->CTL0 |= ADC14_CTL0_SHT0_5 | ADC14_CTL0_SHP | ADC14_CTL0_SSEL_3
                  |ADC14_CTL0_CONSEQ_2 | ADC14_CTL0_ON | ADC14_CTL0_MSC;

    ADC14->MCTL[0] |= ADC14_MCTLN_INCH_6;  // input on A6

    P4->SEL0 |= 0x80; // use A/D
    P4->SEL1 |= 0x80;
    ADC14->CTL0 |= ADC14_CTL0_ENC | ADC14_CTL0_SC;

    return;
}

void main(void)
{
    WDTCTL = WDTPW | WDTHOLD;           // Stop watchdog timer

    // setup port pin for scope timing
    P3->DIR |= BIT6;
    P3->OUT &= ~BIT6;
    init_A2D();
	
    //remove any unnecessary variables
    //In this case all of the variables are being used.
    int arr[4], arr1[100], alpha[4], omega[4];
    int j;
    int x, y, z, beta, gamma, sum = 0;

    while(1)
    {
        beta = ADC14->MEM[0];
        gamma = (beta * beta * beta * beta);
        x = 2;
        y = 62;

        for(j = 0; j < 4; j++)
        {
            alpha[j] = beta * j;
            arr[j] = gamma + (1<<j); //modifcation for i for loop
        }

        //gamma = power(beta, 4);
        //instead of using the power function we can simply multiply
        // the value 4 times
        //gamma = (beta * beta * beta * beta);

        //Run this for loop inside the J for loop instead
        //this will help with speed
        //powers of two is just a bit shifting operator
        /*
        for(i = 0; i < 4; i++)
        {
            x = 2;
            arr[i] = gamma + power(x, i);
        }
        */

        //this can not be modded to the fact the omega k depends on gamm value
        //bit shifted operations.
        //can unroll this for loop tho
        /*
        for(j = 0; j < 4; j++)
        {
            omega[j] = alpha[j] + arr[3 - j];
        }
        */
        omega[0] = alpha[0] + arr[3];
        omega[1] = alpha[1] + arr[2];
        omega[2] = alpha[2] + arr[1];
        omega[3] = alpha[3] + arr[0];

        //hard code this value to the number 62
       // y = x * 31;

        //again hard code this value into a real value
        // (62+3)*4 - (14) = 256
        //z = (y + 3) * 4 - (x + 12);
        z = 256;

        //x does not need to be in the for loop and can also be hard coded
        x = 308;

        int k = 10164; //(33*x)
        //loop into unrolling the loop function
        //reduce the iterations by the amount of 25. Make the function to just
        //loop 4 times
        // could do it 100 times but time is of the essence.
        int i;
        for(i = 0; i < 100; i+=25)
        {
            // replace mod with  power-of-2 bitwise op (x % 2^n == x & (2^n-1))
            // replace unchanging variables with calculated literals
            arr1[i] = k + (y * i) - omega[i & 3];
            arr1[i + 1] = k + (y * (i + 1)) - omega[(i + 1) & 3];
            arr1[i + 2] = k + (y * (i + 2)) - omega[(i + 2) & 3];
            arr1[i + 3] = k + (y * (i + 3)) - omega[(i + 3) & 3];
            arr1[i + 4] = k + (y * (i + 4)) - omega[(i + 4) & 3];
            arr1[i + 5] = k + (y * (i + 5)) - omega[(i + 5) & 3];
            arr1[i + 6] = k + (y * (i + 6)) - omega[(i + 6) & 3];
            arr1[i + 7] = k + (y * (i + 7)) - omega[(i + 7) & 3];
            arr1[i + 8] = k + (y * (i + 8)) - omega[(i + 8) & 3];
            arr1[i + 9] = k + (y * (i + 9)) - omega[(i + 9) & 3];
            arr1[i + 10] = k + (y * (i + 10)) - omega[(i + 10) & 3];
            arr1[i + 11] = k + (y * (i + 11)) - omega[(i + 11) & 3];
            arr1[i + 12] = k + (y * (i + 12)) - omega[(i + 12) & 3];
            arr1[i + 13] = k + (y * (i + 13)) - omega[(i + 13) & 3];
            arr1[i + 14] = k + (y * (i + 14)) - omega[(i + 14) & 3];
            arr1[i + 15] = k + (y * (i + 15)) - omega[(i + 15) & 3];
            arr1[i + 16] = k + (y * (i + 16)) - omega[(i + 16) & 3];
            arr1[i + 17] = k + (y * (i + 17)) - omega[(i + 17) & 3];
            arr1[i + 18] = k + (y * (i + 18)) - omega[(i + 18) & 3];
            arr1[i + 19] = k + (y * (i + 19)) - omega[(i + 19) & 3];
            arr1[i + 20] = k + (y * (i + 20)) - omega[(i + 20) & 3];
            arr1[i + 21] = k + (y * (i + 21)) - omega[(i + 21) & 3];
            arr1[i + 22] = k + (y * (i + 22)) - omega[(i + 22) & 3];
            arr1[i + 23] = k + (y * (i + 23)) - omega[(i + 23) & 3];
            arr1[i + 24] = k + (y * (i + 24)) - omega[(i + 24) & 3];
            arr1[i + 25] = k + (y * (i + 25)) - omega[(i + 25) & 3];
        }

        /*
        for(i = 0; i < 100; i++)
        {
            //x = y + z;
            arr1[i] = (33 * x) + (y * i) -  omega[i%4];
        }
         */
        /*
        for(i = 0; i < 100; i++)
        {
            //remove the mod 9 of this for loop this can be done by just
            // by hardcoding the value
            if((i % 9) == 0)
                sum += arr1[i];
        }
        */
        sum = arr1[0] + arr1[9] + arr1[18] + arr1[27] + arr1[36] + arr1[45]
              + arr1[54] + arr1[63] + arr1[72] + arr1[81] + arr1[90] + arr1[99];

        P3->OUT ^= BIT6;  //toggle for timing measurement
    }
}

//Comment out this power command
/*
int power(int x, int y)
{
    int i, z = 1;
    for(i = 0; i < y; i++)
    {
        z = z * x;
    }
    return z;
}
*/



