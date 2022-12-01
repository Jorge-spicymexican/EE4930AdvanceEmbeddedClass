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

#include <stdint.h>
#include <msp.h>
/* DriverLib Includes */
#include <MSP432P401R_GPIO.h>
#include <MSP432P401R_INTERRUPT.h>

/*
static const uint32_t GPIO_PORT_TO_INT[] =
{
        0x00,
        INT_PORT1,
        INT_PORT2,
        INT_PORT3,
        INT_PORT4,
        INT_PORT5,
        INT_PORT6
};
*/

static const uint32_t GPIO_PORT_TO_BASE[] =
{       0x00,
        0x40004C00,
        0x40004C01,
        0x40004C20,
        0x40004C21,
        0x40004C40,
        0x40004C41,
        0x40004C60,
        0x40004C61,
        0x40004C80,
        0x40004C81,
        0x40004D20
};


// Implements  SEL 0 and SEL 1 with DIR being set
void GPIO_setAsOutputPin(uint_fast8_t selectedPort, uint_fast16_t selectedPins)
{
    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PADIR) |= selectedPins;

    return;
}



void GPIO_setAsInputPin(uint_fast8_t selectedPort, uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PAREN) &= ~selectedPins;
}



void GPIO_setOutputHighOnPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PAOUT) |= selectedPins;
}



void GPIO_setOutputLowOnPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PAOUT) &= ~selectedPins;
}



void GPIO_toggleOutputOnPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PAOUT) ^= selectedPins;
}



void GPIO_setAsInputPinWithPullDownResistor(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;

    HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PAREN) |= selectedPins;
    HWREG16(baseAddress + OFS_PAOUT) &= ~selectedPins;
}



void GPIO_setAsInputPinWithPullUpResistor(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PASEL0) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PASEL1) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PADIR) &= ~selectedPins;
    HWREG16(baseAddress + OFS_PAREN) |= selectedPins;
    HWREG16(baseAddress + OFS_PAOUT) |= selectedPins;
}


uint8_t GPIO_getInputPinValue(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{
        uint16_t inputPinValue;
    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    inputPinValue = HWREG16(baseAddress + OFS_PAIN) & (selectedPins);

    if (inputPinValue > 0)
    {
        return GPIO_INPUT_PIN_HIGH;
    }

    return GPIO_INPUT_PIN_LOW;
}



void GPIO_setDriveStrengthHigh(uint_fast8_t selectedPort,
        uint_fast8_t selectedPins)
{
    uint32_t baseAddr;

    baseAddr = GPIO_PORT_TO_BASE[selectedPort];

    HWREG8(baseAddr + OFS_PADS) |= selectedPins;

}



void GPIO_setDriveStrengthLow(uint_fast8_t selectedPort,
        uint_fast8_t selectedPins)
{
    uint32_t baseAddr;

    baseAddr = GPIO_PORT_TO_BASE[selectedPort];

    HWREG8(baseAddr + OFS_PADS) &= ~selectedPins;

}


void GPIO_enableInterrupt(uint_fast8_t selectedPort, uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PAIE) |= selectedPins;
}


void GPIO_disableInterrupt(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    HWREG16(baseAddress + OFS_PAIE) &= ~selectedPins;
}


uint_fast16_t GPIO_getInterruptStatus(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];

    return HWREG16(baseAddress + OFS_PAIFG) & selectedPins;
}


void GPIO_clearInterruptFlag(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];


    HWREG16(baseAddress + OFS_PAIFG) &= ~selectedPins;
}


void GPIO_interruptEdgeSelect(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins, uint_fast8_t edgeSelect)
{

    uint32_t baseAddress = GPIO_PORT_TO_BASE[selectedPort];


    if (GPIO_LOW_TO_HIGH_TRANSITION == edgeSelect)
        HWREG16(baseAddress + OFS_PAIES) &= ~selectedPins;
    else
        HWREG16(baseAddress + OFS_PAIES) |= selectedPins;
}

uint_fast16_t GPIO_getEnabledInterruptStatus(uint_fast8_t selectedPort)
{
    uint_fast16_t pendingInts;
    uint32_t baseAddr;

    pendingInts = GPIO_getInterruptStatus(selectedPort, 0xFFFF);
    baseAddr = GPIO_PORT_TO_BASE[selectedPort];

    switch (selectedPort)
    {
    case GPIO_PORT_P1:
    case GPIO_PORT_P3:
    case GPIO_PORT_P5:
    case GPIO_PORT_P7:
    case GPIO_PORT_P9:
        return (HWREG8(baseAddr + OFS_P1IE) & pendingInts);
    case GPIO_PORT_P2:
    case GPIO_PORT_P4:
    case GPIO_PORT_P6:
    case GPIO_PORT_P8:
    case GPIO_PORT_P10:
        return (HWREG8(baseAddr + OFS_P2IE) & pendingInts);
    case GPIO_PORT_PJ:
        return (HWREG16(baseAddr + OFS_PAIE) & pendingInts);
    default:
        return 0;
    }
}



/*void GPIO_registerInterrupt(uint_fast8_t selectedPort, void (*intHandler)(void))
{
    uint32_t wPortInt;

    wPortInt = GPIO_PORT_TO_INT[selectedPort];

    //
    // Register the interrupt handler, returning an error if an error occurs.
    //
    Interrupt_registerInterrupt(wPortInt, intHandler);

    //
    // Enable the system control interrupt.
    //
    Interrupt_enableInterrupt(wPortInt);
}


void GPIO_unregisterInterrupt(uint_fast8_t selectedPort)
{
    uint32_t wPortInt;

    wPortInt = GPIO_PORT_TO_INT[selectedPort];

    //
    // Disable the interrupt.
    //
    Interrupt_disableInterrupt(wPortInt);

    //
    // Unregister the interrupt handler.
    //
    Interrupt_unregisterInterrupt(wPortInt);
}
*/

