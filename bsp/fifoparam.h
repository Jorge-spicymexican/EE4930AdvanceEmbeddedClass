/* Copyright (C) 2017 TCI LLC
 *
 * All Rights Reserved
 * You may not use, distribute or modify this code without the
 * express written permission of TCI, LLC
 *
 * Contact Info
 * www.transcoil.com
 * 414-357-4480
 * tech-support@transcoil.com
 * TCI, LLC
 * Germantown WI, 53022 USA
 *
 */


#ifndef fifoparam_h
#define fifoparam_h

#include"defines.h"

//defines
#define FIFO_PARAM_BUFFER_SIZE     64

//module data
typedef struct
{
    Uint16 usAddress;
    Uint16 usData;
}
FIFO_PARAM_BUFFER_TYPE;



typedef struct
{
    //write FIFO buffer
    Uint16 usFifoParamBufferHead;
    Uint16 usFifoParamBufferTail;
    FIFO_PARAM_BUFFER_TYPE asFifoParamBuffer[FIFO_PARAM_BUFFER_SIZE];
}
FIFO_PARAM_TYPE;


//function prototypes
void FifoParamInit(FIFO_PARAM_TYPE* psParamFifo );
void FifoParamQue(FIFO_PARAM_TYPE* psParamFifo, FIFO_PARAM_BUFFER_TYPE sParam);
void FifoParamDeque(FIFO_PARAM_TYPE* psParamFifo, FIFO_PARAM_BUFFER_TYPE* psParam);

//inline functions
inline Bit FifoParamEmpty( FIFO_PARAM_TYPE* psParamFifo )
{
    return psParamFifo->usFifoParamBufferHead == psParamFifo->usFifoParamBufferTail ? TRUE : FALSE;
}

#endif
