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

#ifndef DRIVERS_MSP432P401R_GPIO_H_
#define DRIVERS_MSP432P401R_GPIO_H_


//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

#include <msp.h>
#include <stdint.h>

#define GPIO_PORT_P1                                                          1
#define GPIO_PORT_P2                                                          2
#define GPIO_PORT_P3                                                          3
#define GPIO_PORT_P4                                                          4
#define GPIO_PORT_P5                                                          5
#define GPIO_PORT_P6                                                          6
#define GPIO_PORT_P7                                                          7
#define GPIO_PORT_P8                                                          8
#define GPIO_PORT_P9                                                          9
#define GPIO_PORT_P10                                                         10
#define GPIO_PORT_PJ                                                         11

// Do not use the PIN 8 or above
#define GPIO_PIN0                                                      (0x0001)
#define GPIO_PIN1                                                      (0x0002)
#define GPIO_PIN2                                                      (0x0004)
#define GPIO_PIN3                                                      (0x0008)
#define GPIO_PIN4                                                      (0x0010)
#define GPIO_PIN5                                                      (0x0020)
#define GPIO_PIN6                                                      (0x0040)
#define GPIO_PIN7                                                      (0x0080)
#define GPIO_PIN8                                                      (0x0100)
#define GPIO_PIN9                                                      (0x0200)
#define GPIO_PIN10                                                     (0x0400)
#define GPIO_PIN11                                                     (0x0800)
#define GPIO_PIN12                                                     (0x1000)
#define GPIO_PIN13                                                     (0x2000)
#define GPIO_PIN14                                                     (0x4000)
#define GPIO_PIN15                                                     (0x8000)
#define PIN_ALL8                                                       (0xFF)
#define PIN_ALL16

#define GPIO_HIGH_TO_LOW_TRANSITION                                      (0x01)
#define GPIO_LOW_TO_HIGH_TRANSITION                                      (0x00)

#define GPIO_INPUT_PIN_HIGH                                              (0x01)
#define GPIO_INPUT_PIN_LOW                                               (0x00)

//*****************************************************************************
//
//! \brief This function configures the selected Pin as output pin
//!
//! This function selected pins on a selected port as output pins.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register and bits of \b PxSEL register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setAsOutputPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);


//*****************************************************************************
//
//! \brief This function configures the selected Pin as input pin
//!
//! This function selected pins on a selected port as input pins.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register, bits of \b PxREN register and bits of
//! \b PxSEL register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setAsInputPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);


//*****************************************************************************
//
//! This function sets the drive strength to high for the selected port
//!
//!
//! \param selectedPort is the selected port.
//!             Valid values are:
//!             - \b GPIO_PORT_P1,
//!             - \b GPIO_PORT_P2,
//!             - \b GPIO_PORT_P3,
//!             - \b GPIO_PORT_P4,
//!             - \b GPIO_PORT_P5,
//!             - \b GPIO_PORT_P6,
//!             - \b GPIO_PORT_P7,
//!             - \b GPIO_PORT_P8,
//!             - \b GPIO_PORT_P9,
//!             - \b GPIO_PORT_P10,
//!             - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are:
//!             - \b GPIO_PIN0,
//!             - \b GPIO_PIN1,
//!             - \b GPIO_PIN2,
//!             - \b GPIO_PIN3,
//!             - \b GPIO_PIN4,
//!             - \b GPIO_PIN5,
//!             - \b GPIO_PIN6,
//!             - \b GPIO_PIN7,
//!             - \b GPIO_PIN8,
//!             - \b PIN_ALL8,
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setDriveStrengthHigh(uint_fast8_t selectedPort,
        uint_fast8_t selectedPins);

//*****************************************************************************
//
//! This function sets the drive strength to low for the selected port
//!
//!
//! \param selectedPort is the selected port.
//!             Valid values are:
//!             - \b GPIO_PORT_P1,
//!             - \b GPIO_PORT_P2,
//!             - \b GPIO_PORT_P3,
//!             - \b GPIO_PORT_P4,
//!             - \b GPIO_PORT_P5,
//!             - \b GPIO_PORT_P6,
//!             - \b GPIO_PORT_P7,
//!             - \b GPIO_PORT_P8,
//!             - \b GPIO_PORT_P9,
//!             - \b GPIO_PORT_P10,
//!             - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!             Valid values are:
//!             - \b GPIO_PIN0,
//!             - \b GPIO_PIN1,
//!             - \b GPIO_PIN2,
//!             - \b GPIO_PIN3,
//!             - \b GPIO_PIN4,
//!             - \b GPIO_PIN5,
//!             - \b GPIO_PIN6,
//!             - \b GPIO_PIN7,
//!             - \b GPIO_PIN8,
//!             - \b PIN_ALL8,
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setDriveStrengthLow(uint_fast8_t selectedPort,
        uint_fast8_t selectedPins);

//*****************************************************************************
//
//! \brief This function sets output HIGH on the selected Pin
//!
//! This function sets output HIGH on the selected port's pin.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxOUT register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setOutputHighOnPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function sets output LOW on the selected Pin
//!
//! This function sets output LOW on the selected port's pin.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxOUT register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setOutputLowOnPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function toggles the output on the selected Pin
//!
//! This function toggles the output on the selected port's pin.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxOUT register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_toggleOutputOnPin(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function sets the selected Pin in input Mode with Pull Down
//! resistor
//!
//! This function sets the selected Pin in input Mode with Pull Down resistor.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register, bits of \b PxOUT register and bits of
//! \b PxREN register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setAsInputPinWithPullDownResistor(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function sets the selected Pin in input Mode with Pull Up
//! resistor
//!
//! This function sets the selected Pin in input Mode with Pull Up resistor.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxDIR register, bits of \b PxOUT register and bits of
//! \b PxREN register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_setAsInputPinWithPullUpResistor(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function gets the input value on the selected pin
//!
//! This function gets the input value on the selected pin.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Valid values are:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! \return One of the following:
//!         - \b GPIO_INPUT_PIN_HIGH
//!         - \b GPIO_INPUT_PIN_LOW
//!         \n indicating the status of the pin
//
//*****************************************************************************
extern uint8_t GPIO_getInputPinValue(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);




//*****************************************************************************
extern void GPIO_enableInterrupt(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function disables the port interrupt on the selected pin
//!
//! This function disables the port interrupt on the selected pin. Note that
//! only Port 1,2, A have this capability.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_PA
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxIE register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_disableInterrupt(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function gets the interrupt status of the selected pin
//!
//! This function gets the interrupt status of the selected pin. Note that only
//! Port 1,2, A have this capability.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_PA
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! \return Logical OR of any of the following:
//!         - \b GPIO_PIN0
//!         - \b GPIO_PIN1
//!         - \b GPIO_PIN2
//!         - \b GPIO_PIN3
//!         - \b GPIO_PIN4
//!         - \b GPIO_PIN5
//!         - \b GPIO_PIN6
//!         - \b GPIO_PIN7
//!         - \b GPIO_PIN8
//!         - \b GPIO_PIN9
//!         - \b GPIO_PIN10
//!         - \b GPIO_PIN11
//!         - \b GPIO_PIN12
//!         - \b GPIO_PIN13
//!         - \b GPIO_PIN14
//!         - \b GPIO_PIN15
//!         \n indicating the interrupt status of the selected pins [Default:
//!         0]
//
//*****************************************************************************
extern uint_fast16_t GPIO_getInterruptStatus(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function clears the interrupt flag on the selected pin
//!
//! This function clears the interrupt flag on the selected pin. Note that only
//! Port 1,2,A have this capability.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_PA
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//!
//! Modified bits of \b PxIFG register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_clearInterruptFlag(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins);

//*****************************************************************************
//
//! \brief This function selects on what edge the port interrupt flag should be
//! set for a transition
//!
//! This function selects on what edge the port interrupt flag should be set
//! for a transition. Values for edgeSelect should be
//! GPIO_LOW_TO_HIGH_TRANSITION or GPIO_HIGH_TO_LOW_TRANSITION.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//! \param selectedPins is the specified pin in the selected port.
//!        Mask value is the logical OR of any of the following:
//!        - \b GPIO_PIN0
//!        - \b GPIO_PIN1
//!        - \b GPIO_PIN2
//!        - \b GPIO_PIN3
//!        - \b GPIO_PIN4
//!        - \b GPIO_PIN5
//!        - \b GPIO_PIN6
//!        - \b GPIO_PIN7
//!        - \b GPIO_PIN8
//!        - \b GPIO_PIN9
//!        - \b GPIO_PIN10
//!        - \b GPIO_PIN11
//!        - \b GPIO_PIN12
//!        - \b GPIO_PIN13
//!        - \b GPIO_PIN14
//!        - \b GPIO_PIN15
//! \param edgeSelect specifies what transition sets the interrupt flag
//!        Valid values are:
//!        - \b GPIO_HIGH_TO_LOW_TRANSITION
//!        - \b GPIO_LOW_TO_HIGH_TRANSITION
//!
//! Modified bits of \b PxIES register.
//!
//! \return None
//
//*****************************************************************************
extern void GPIO_interruptEdgeSelect(uint_fast8_t selectedPort,
        uint_fast16_t selectedPins, uint_fast8_t edgeSelect);

//*****************************************************************************
//
//! \brief This function gets the interrupt status of the provided PIN and
//!         masks it with the interrupts that are actually enabled. This is
//!         useful for inside ISRs where the status of only the enabled
//!         interrupts needs to be checked.
//!
//! \param selectedPort is the selected port.
//!        Valid values are:
//!        - \b GPIO_PORT_P1
//!        - \b GPIO_PORT_P2
//!        - \b GPIO_PORT_P3
//!        - \b GPIO_PORT_P4
//!        - \b GPIO_PORT_P5
//!        - \b GPIO_PORT_P6
//!        - \b GPIO_PORT_P7
//!        - \b GPIO_PORT_P8
//!        - \b GPIO_PORT_P9
//!        - \b GPIO_PORT_P10
//!        - \b GPIO_PORT_P11
//!        - \b GPIO_PORT_PJ
//!
//! \return Logical OR of any of the following:
//!         - \b GPIO_PIN0
//!         - \b GPIO_PIN1
//!         - \b GPIO_PIN2
//!         - \b GPIO_PIN3
//!         - \b GPIO_PIN4
//!         - \b GPIO_PIN5
//!         - \b GPIO_PIN6
//!         - \b GPIO_PIN7
//!         - \b GPIO_PIN8
//!         - \b GPIO_PIN9
//!         - \b GPIO_PIN10
//!         - \b GPIO_PIN11
//!         - \b GPIO_PIN12
//!         - \b GPIO_PIN13
//!         - \b GPIO_PIN14
//!         - \b GPIO_PIN15,
//!         - \b PIN_ALL8,
//!         - \b PIN_ALL16
//!         \n indicating the interrupt status of the selected pins [Default:
//!         0]
//
//*****************************************************************************
extern uint_fast16_t GPIO_getEnabledInterruptStatus(uint_fast8_t selectedPort);


//*****************************************************************************
//
//! Registers an interrupt handler for the port interrupt.
//!
//! \param selectedPort is the port to register the interrupt handler
//!
//! \param intHandler is a pointer to the function to be called when the port
//! interrupt occurs.
//!
//! This function registers the handler to be called when a port
//! interrupt occurs. This function enables the global interrupt in the
//! interrupt controller; specific GPIO interrupts must be enabled
//! via GPIO_enableInterrupt().  It is the interrupt handler's responsibility to
//! clear the interrupt source via GPIO_clearInterruptFlag().
//!
//! Clock System can generate interrupts when
//!
//! \sa Interrupt_registerInterrupt() for important information about
//! registering interrupt handlers.
//!
//! \return None.
//
//*****************************************************************************
extern void GPIO_registerInterrupt(uint_fast8_t selectedPort,
        void (*intHandler)(void));

//*****************************************************************************
//
//! Unregisters the interrupt handler for the port.
//!
//! \param selectedPort is the port to unregister the interrupt handler
//!
//! This function unregisters the handler to be called when a port
//! interrupt occurs.  This function also masks off the interrupt in the
//! interrupt controller so that the interrupt handler no longer is called.
//!
//! \sa Interrupt_registerInterrupt() for important information about
//! registering interrupt handlers.
//!
//! \return None.
//
//*****************************************************************************
extern void GPIO_unregisterInterrupt(uint_fast8_t selectedPort);


//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

//*****************************************************************************
//
// Close the Doxygen group.
//! @}
//
//*****************************************************************************


#endif /* DRIVERS_MSP432P401R_GPIO_H_ */
