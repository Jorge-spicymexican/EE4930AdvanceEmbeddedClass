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
#include "EE4930_LAB1.h"

/*
 * main.c
 */

int main(void){

     Init_Lab1();

    while(1)
    {
        Lab1_Poll();
    }


} // end main
