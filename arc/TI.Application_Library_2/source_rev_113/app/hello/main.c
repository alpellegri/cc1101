/******************************************************************************
    Filename: main.c

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_uart.h>
#include <hal_lcd.h>
#include <hal_led.h>
#include <hal_mcu.h>
#include <hal_spi.h>
#include <hal_board.h>
#include <hal_rf.h>

//----------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//  This is the main entry of the "hello" application. The only thing this
//  application does is to display the partnumber and version of the RF chip on
//  the attached Chipcon EM. Write a string to the UART interface at a regular
//  interval.
//----------------------------------------------------------------------------------
void main (void)
{
    uint16 counter;
    uint8  id;
    uint8  ver;

    halBoardInit();

    // Reset chip
    halRfResetChip();

    // Display partnumber and version on the LCD
    id  = halRfGetChipId();
    ver = halRfGetChipVer();
    halLcdWriteValue((id << 8) | ver, HAL_LCD_RADIX_HEX, 0);
    
    // Write something to the UART
    counter = 0;
    while(TRUE)
    {
        halLedToggle(++counter%3 + 1);
        halMcuWaitUs(50000);
        halUartWrite("Chipcon products from Texas Instruments. Connecting smarter!", counter%60 + 1);
        halUartWrite("\r\n", 2);
    }
}

