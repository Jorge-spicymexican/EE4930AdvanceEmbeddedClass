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

#ifndef defines     
#define defines


//#define  WORD unsigned short
//#define  BYTE unsigned char

#ifndef TRUE
#define TRUE 1
#endif

#ifndef PRESSED
#define PRESSED 0
#endif

#ifndef NOT_PRESSED
#define NOT_PRESSED 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#ifndef NULL
#define NULL 0
#endif
#define E 2.71828182
#define PI 3.14156

typedef int                 int16;
typedef long                int32;
typedef long long           int64;
typedef unsigned int        Uint16;
typedef unsigned long       Uint32;
typedef unsigned long long  Uint64;

typedef float               float32;
typedef long double         float64;

//leave bits at Uint32 as some of them are setpoint parameters (params expect 32 bit variables)
typedef unsigned long   Bit;


#endif

/*
 *  © 2022 MSOE
 */
