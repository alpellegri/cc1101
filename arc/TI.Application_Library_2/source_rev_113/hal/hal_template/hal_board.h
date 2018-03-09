/******************************************************************************
    Filename: hal_board.h

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#ifndef HAL_BOARD_H
#define HAL_BOARD_H

#include "hal_types.h"
#include "hal_defs.h"
#include "hal_digio.h"

//----------------------------------------------------------------------------------
// This is where you include the header file defining all of the registers
// for your specific microcontroller.
//----------------------------------------------------------------------------------
#include <msp430xG46x.h>
#include "hal_msp430.h"


//----------------------------------------------------------------------------------
//  Easy access, zero overhead LED macros
//  Adapted to hardware interface on MSP430 Experimenter Board
//----------------------------------------------------------------------------------
#define HAL_LED_SET_1   (P1OUT |= BIT1)
#define HAL_LED_SET_2   (P1OUT |= BIT2)

#define HAL_LED_CLR_1   (P1OUT &= ~BIT1)
#define HAL_LED_CLR_2   (P1OUT &= ~BIT2)

#define HAL_LED_TGL_1   (P1OUT ^= BIT1)
#define HAL_LED_TGL_2   (P1OUT ^= BIT2)



//----------------------------------------------------------------------------------
//  Port and pin where GDO0 and GDO2 from CC1100/CC2500 are connected
//----------------------------------------------------------------------------------
#define HAL_IO_GDO0_PORT  1
#define HAL_IO_GDO0_PIN   3
#define HAL_IO_GDO2_PORT  1
#define HAL_IO_GDO2_PIN   4


//----------------------------------------------------------------------------------
//  Define ports and pins used by SPI interface to CC1100/CC2500
//----------------------------------------------------------------------------------
#define HAL_SPI_SOMI_PORT 4
#define HAL_SPI_SOMI_PIN  4
#define HAL_SPI_SIMO_PORT 4
#define HAL_SPI_SIMO_PIN  3
#define HAL_SPI_CLK_PORT  4
#define HAL_SPI_CLK_PIN   5
#define HAL_SPI_CS_PORT   4
#define HAL_SPI_CS_PIN    2

//----------------------------------------------------------------------------------
// Select interface on MSP430 to use for SPI (define only one!)
//----------------------------------------------------------------------------------

// #define HAL_SPI_INTERFACE_USART0
#define HAL_SPI_INTERFACE_USART1
// #define HAL_SPI_INTERFACE_USCIA0
// #define HAL_SPI_INTERFACE_USCIA1
// #define HAL_SPI_INTERFACE_USCIB0
// #define HAL_SPI_INTERFACE_USCIB1
// #define HAL_SPI_INTERFACE_USI
// #define HAL_SPI_INTERFACE_BITBANG



#ifdef __cplusplus
extern "C" {
#endif


extern const digioConfig pinGDO0;
extern const digioConfig pinGDO2;



//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void halBoardInit(void);


#ifdef __cplusplus
}
#endif

/**********************************************************************************/
#endif
