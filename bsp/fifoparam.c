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

#include "fifoparam.h"

//function prototypes
void FifoParamInit(FIFO_PARAM_TYPE* psParamFifo );
void FifoParamQue(FIFO_PARAM_TYPE* psParamFifo, FIFO_PARAM_BUFFER_TYPE sParam);
void FifoParamDeque(FIFO_PARAM_TYPE* psParamFifo, FIFO_PARAM_BUFFER_TYPE* psParam);


void FifoParamInit(FIFO_PARAM_TYPE* psParamFifo )
{
    //check for null pointer
    if(psParamFifo != 0)
    {
        psParamFifo->usFifoParamBufferHead = 0;
        psParamFifo->usFifoParamBufferTail = 0;
    }
}

void FifoParamQue(FIFO_PARAM_TYPE* psParamFifo, FIFO_PARAM_BUFFER_TYPE sParam)
{
    Uint16 usNextIndex;

    //get the next open slot in the queue
    usNextIndex = psParamFifo->usFifoParamBufferHead + 1;

    //roll over FIFO que if needed
    if(usNextIndex >= FIFO_PARAM_BUFFER_SIZE)
        usNextIndex = 0;

    //enqueue the ucData only if there is room
    if(usNextIndex != psParamFifo->usFifoParamBufferTail)
    {
        //store data to the fifo
        psParamFifo->asFifoParamBuffer[psParamFifo->usFifoParamBufferHead].usAddress = sParam.usAddress;
        psParamFifo->asFifoParamBuffer[psParamFifo->usFifoParamBufferHead].usData = sParam.usData;

        //update head index
        psParamFifo->usFifoParamBufferHead = usNextIndex;
    }
}

void FifoParamDeque(FIFO_PARAM_TYPE* psParamFifo, FIFO_PARAM_BUFFER_TYPE* psParam)
{
    //null pointer check
    if(psParamFifo==0 || psParam==0)
        return;

    //check if buffer has an entry to dequeue
    if (psParamFifo->usFifoParamBufferHead != psParamFifo->usFifoParamBufferTail)
    {
        Uint16 usNextIndex;

        //figure out the next tail index post dequeue
        usNextIndex = psParamFifo->usFifoParamBufferTail + 1;

        //roll over read index if needed
        if(usNextIndex >= FIFO_PARAM_BUFFER_SIZE)
            usNextIndex = 0;

        //get data from fifo
        psParam->usAddress = psParamFifo->asFifoParamBuffer[psParamFifo->usFifoParamBufferTail].usAddress;
        psParam->usData = psParamFifo->asFifoParamBuffer[psParamFifo->usFifoParamBufferTail].usData;

        //update the FIFO read index
        psParamFifo->usFifoParamBufferTail = usNextIndex;
    }
}








