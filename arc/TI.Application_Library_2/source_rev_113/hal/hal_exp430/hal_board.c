/***********************************************************************************
    Filename: hal_board.c

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include "hal_types.h"
#include "hal_defs.h"
#include "hal_digio.h"
#include "hal_int.h"
#include "hal_mcu.h"
#include "hal_board.h"
#include "hal_lcd.h"
#include "hal_spi.h"
#include "hal_uart.h"

//------------------------------------------------------------------------------
//  Global variables
//------------------------------------------------------------------------------

// The constants below define some of the I/O signals used by the board
// Port, pin number, pin bitmask, direction and initial value should be
// set in order to match the target hardware. Once defined, the pins are
// configured in halBoardInit() by calling halDigioConfig()

const digioConfig pinLed1   = {2, 1, BIT1, HAL_DIGIO_OUTPUT, 0};
const digioConfig pinLed2   = {2, 2, BIT2, HAL_DIGIO_OUTPUT, 0};
const digioConfig pinLed4   = {5, 1, BIT1, HAL_DIGIO_OUTPUT, 0};
const digioConfig pinS1     = {1, 0, BIT0, HAL_DIGIO_INPUT,  0};
const digioConfig pinS2     = {1, 1, BIT1, HAL_DIGIO_INPUT,  0};
const digioConfig pinGDO0   = {1, 2, BIT2, HAL_DIGIO_INPUT,  0};
const digioConfig pinGDO2   = {1, 3, BIT3, HAL_DIGIO_INPUT,  0};


//------------------------------------------------------------------------------
//  void halBoardInit(void)
//
//  DESCRIPTION:
//    Set up board. Initialize MCU, configure I/O pins and user interfaces
//------------------------------------------------------------------------------
void halBoardInit(void)
{
    halMcuInit();

    halDigioConfig(&pinLed1);
    halDigioConfig(&pinLed2);
    halDigioConfig(&pinLed4);
    halDigioConfig(&pinS1);
    halDigioConfig(&pinS2);
    halDigioConfig(&pinGDO0);
    halDigioConfig(&pinGDO2);

    halLcdInit();
    halSpiInit(0);
    halUartInit(0, 0);

    halIntOn();
}
