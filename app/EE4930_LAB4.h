/*
 * EE4930_LAB4.h
 *
 *  Created on: Jan 2, 2023
 *      Author: jurado-garciaj
 *      Run a timer to read the Freezer inputs to see when to two tick
 */

#ifndef APP_EE4930_LAB4_H_
#define APP_EE4930_LAB4_H_

#define FALSE   0
#define TRUE    1
#define OFF     0
#define ON      1

#define TICKS_MAX 9
#define LOOPUP_COLUMN 4
#define LOOPUP_ROW 3


//Different states
typedef enum
{
    Off_state,
    Defrost_state,
    Active_state,
    last_state
} eSystemState;


//Different type events
typedef enum
{
    Humidity_above_setpoint_Ice_off_event,
    Humidity_below_setpoint_Ice_off_event,
    Humidity_within_setpoint_Ice_off_event,
    Ice_sensed_event,
    Ice_off_event,
    Last_event
} eSystemEvent;


//input structure  of state machine
typedef struct
{
    unsigned char Room_temperature;
    unsigned char Humidity_percentage;
    unsigned char Humidity_setpoints;
    char Ice_sensor;

} eSystemInputs;


//output structure of state machine
typedef struct
{
    char Fan_control;
    char Compressor;
    char LCD_Display_flag;

} eSystemOutputs;


//typedef of function pointer
typedef eSystemState (*pfEventHandler)(void);


//structure of state and event with event handler
typedef struct
{
    eSystemState eStateMachine;
    eSystemEvent eStateMachineEvent;
    pfEventHandler pfStateMachineEvnentHandler;

} sStateMachine;


//structure of module itself
typedef struct
{
    eSystemInputs sInputs;
    eSystemOutputs sOutputs;
    eSystemState eCurrentState;
    eSystemState ePreviousState;
    char Errorcode;
} sDehumidifier;



void Dehumidifier_Read( eSystemInputs inputs );
eSystemEvent Dehumidifier_ReadEvent( void );
eSystemState Dehumidifier_Poll( eSystemEvent NewEvent, eSystemState NextState);
void Dehumidifier_init( void );





#endif /* APP_EE4930_LAB4_H_ */
