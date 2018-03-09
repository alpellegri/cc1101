/******************************************************************************
    Filename: hal_msp430.h

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#ifndef HAL_MSP430_H
#define HAL_MSP430_H

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_board.h>


//----------------------------------------------------------------------------------
//  Macros for simple configuration of IO pins on MSP430
//----------------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL(port, pin)   MCU_IO_PERIPHERAL_PREP(port, pin)
#define MCU_IO_INPUT(port, pin)        MCU_IO_INPUT_PREP(port, pin)
#define MCU_IO_OUTPUT(port, pin, val)  MCU_IO_OUTPUT_PREP(port, pin, val)
#define MCU_IO_SET(port, pin)          MCU_IO_SET_PREP(port, pin)
#define MCU_IO_CLR(port, pin)          MCU_IO_CLR_PREP(port, pin)
#define MCU_IO_GET(port, pin)          MCU_IO_GET_PREP(port, pin)




//----------------------------------------------------------------------------------
//  Macros for internal use (the macros above need a new round in the preprocessor)
//----------------------------------------------------------------------------------
#define MCU_IO_PERIPHERAL_PREP(port, pin)  st( P##port##SEL |= BIT##pin##; )
#define MCU_IO_INPUT_PREP(port, pin)       st( P##port##SEL &= ~BIT##pin##; \
                                               P##port##DIR &= ~BIT##pin##; )
#define MCU_IO_OUTPUT_PREP(port, pin, val) st( P##port##SEL &= ~BIT##pin##; \
                                               if (val) \
                                                   { P##port##OUT |= BIT##pin##; } \
                                               else \
                                                   { P##port##OUT &= ~BIT##pin##; } \
                                               P##port##DIR |= BIT##pin##; )
#define MCU_IO_SET_PREP(port, pin)         st( P##port##OUT |= BIT##pin##; )
#define MCU_IO_CLR_PREP(port, pin)         st( P##port##OUT &= ~BIT##pin##; )
#define MCU_IO_GET_PREP(port, pin)         (P##port##IN & BIT##pin##)




/**********************************************************************************/
#endif
