/***********************************************************************************
    Filename: hal_uart.c

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include "hal_types.h"
#include "hal_uart.h"
#include "hal_board.h"


//----------------------------------------------------------------------------------
//  void halUartInit(uint8 baudrate, uint8 options)
//----------------------------------------------------------------------------------
void halUartInit(uint8 baudrate, uint8 options)
{
    // For the moment, this UART implementation only
    // supports communication settings 115200 8N1
    // i.e. ignore baudrate and options arguments.

    UCA0CTL1 |= UCSWRST;               // Keep USCI in reset state
    UCA0CTL1 |= UCSSEL_2;              // SMCLK
    UCA0BR0  = 0x22;                   // 4MHz 115200
    UCA0BR1  = 0x00;                   // 4MHz 115200
    UCA0MCTL = 0x08;                   // 4Mhz Modulation
    
    // Set up pins used by peripheral unit (USCI_A0)
    P2SEL |= BIT4;    // P2.4 = TXD

    UCA0CTL1 &= ~UCSWRST;              // Initialize USCI state machine
}

//----------------------------------------------------------------------------------
//  void halUartWrite(const uint8* buf, uint16 length)
//----------------------------------------------------------------------------------
void halUartWrite(const uint8* buf, uint16 length)
{
    uint16 i;
    for(i = 0; i < length; i++)
    {
        while (!(IFG2 & UCA0TXIFG));   // Wait for TX buffer ready to receive new byte
        UCA0TXBUF = buf[i];            // Output character
    }
}

//----------------------------------------------------------------------------------
//  void halUartRead(uint8* buf, uint16 length)
//----------------------------------------------------------------------------------
void halUartRead(uint8* buf, uint16 length)
{
}

