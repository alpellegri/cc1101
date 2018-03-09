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

const digioConfig pinGDO0   = {1, 2, BIT2, HAL_DIGIO_INPUT, 0};
const digioConfig pinGDO2   = {1, 3, BIT3, HAL_DIGIO_INPUT, 0};


//------------------------------------------------------------------------------
//  void halBoardInit(void)
//
//  DESCRIPTION:
//    Set up board. Initialize MCU, configure I/O pins and user interfaces
//------------------------------------------------------------------------------
void halBoardInit(void)
{
    // Configure MCU (set up clock system etc.)
    halMcuInit();

    // Configure digital I/O pins
    halDigioConfig(&pinGDO0);
    halDigioConfig(&pinGDO2);

    // Optionally, you can configure I/O pins using these macros
    MCU_IO_OUTPUT(1, 0, 0); // LED 1 (port 1, pin 0, initial state off)
    MCU_IO_OUTPUT(1, 1, 0); // LED 2 (port 1, pin 1, initial state off)

    // Initialize SPI interface
    halSpiInit(0);
    
    // Add other initialization functions here

}
