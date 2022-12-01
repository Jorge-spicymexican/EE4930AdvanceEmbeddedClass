/*
 * MSP432P401R_DEBUG.h
 *
 *  Created on: Dec 1, 2022
 *      Author: jurado-garciaj
 */

#ifndef DRIVERS_MSP432P401R_DEBUG_H_
#define DRIVERS_MSP432P401R_DEBUG_H_

//*****************************************************************************
//
// Prototype for the function that is called when an invalid argument is passed
// to an API.  This is only used when doing a DEBUG build.
//
//*****************************************************************************
extern void __error__(char *pcFilename, unsigned long line);

//*****************************************************************************
//
// The ASSERT macro, which does the actual assertion checking.  Typically, this
// will be for procedure arguments.
//
//*****************************************************************************
#ifdef DEBUG
#define ASSERT(expr) {                                      \
                         if(!(expr))                        \
                         {                                  \
                             __error__(__FILE__, __LINE__); \
                         }                                  \
                     }
#else
#define ASSERT(expr)
#endif

#ifdef DEBUG
#define assert(expr) {                                      \
                         if(!(expr))                        \
                         {                                  \
                             __error__(__FILE__, __LINE__); \
                         }                                  \
                     }
#else
#define assert(expr)
#endif



#endif /* DRIVERS_MSP432P401R_DEBUG_H_ */
