/*
 * EE4930_LAB5.h
 *
 *  Created on: Jan 22, 2023
 *      Author: jurado-garciaj
Specifications:

The code must use the A/D to measure temperature in degrees F
(resolution of 0.1 deg. F min.)

using a TMP36 sensor. The temperature reading must be stored in a global
variable that the wireless radio can access, and the radio must be
notified that a new value is available for transmission (e.g., set an output
high for some short time interval, then back low).

Temperature readings must be taken about every ten minutes and transmitted
to the base station.

The battery is a CR2032 lithium battery with a capacity of about 210 mAhr.

Use the Energy Trace tool to analyze your code in terms of its time spent in various modes.

Use a series resistor (e.g., 10 Ohms) in the power connection to measure the
actual power supply current of the system on an oscilloscope

Calculate the estimated battery life of your system using both the Energy Trace data and the scope data.

For code demo, set the time between readings to something more reasonable, like 10 seconds.
 */

#ifndef APP_EE4930_LAB5_H_
#define APP_EE4930_LAB5_H_

//use an ADC converter to measure the temp connect to Pin5.2
void Init_Temp( void );

void Init_Watchdog( void);
void init_CS( void );
void Init_PCM( void );



#endif /* APP_EE4930_LAB5_H_ */
