/*
 * EE4930_FINAL_EXAM.h
 *
 *  Created on: Feb 14, 2023
 *      Author: jurado-garciaj
 */

#ifndef APP_EE4930_FINAL_EXAM_H_
#define APP_EE4930_FINAL_EXAM_H_

#include "defines.h"

//defs
#define LOOPUP_COLUMN 2
#define LOOPUP_ROW 2


//Different states
typedef enum
{
    Off_state,
    Active_state,
    last_state
} eSystemState;


//Different type events
typedef enum
{
    Temp_above_setpoint_event,
    Temp_below_setpoint_event,
    no_new_event,
    Last_event
} eSystemEvent;


//input structure  of state machine
typedef struct
{
    signed short Igloo_temp;
    signed short setpoint_temp;
} eSystemInputs;


//output structure of state machine
typedef struct
{
    char Fan_control;
    char Heating_Coil;
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
} SIgloo;


void Igloo_init( void );
int Config_Duty( float duty );
eSystemEvent Igloo_ReadEvent( void );
eSystemState Igloo_Poll( eSystemEvent NewEvent, eSystemState NextState);
void Igloo_UpdateLCD( void );


#endif /* APP_EE4930_FINAL_EXAM_H_ */
