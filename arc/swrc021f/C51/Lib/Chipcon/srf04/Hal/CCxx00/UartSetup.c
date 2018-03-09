/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements uart configuration for the c8051f320 ports            *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     UartSetup.c                                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320                                                *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:        See end of file                                                            *
 ******************************************************************************************************/
#include <Chipcon\srf04\halsrf04.h>




//------------------------------------------------------------------------------------------------------
//  void halUartSetup(UINT16 baudRate, UINT8 options)
//
//  DESCRIPTION:
//      Function which implements all the initialization necessary to establish a simple serial link. 
//      Timer1 is used as a baudrate generator and is initialized according to _baudRate_. Timer 1 
//      is enabled and configures for Mode 2; 8-bit counter/timer with auto-reload. The UART 
//      is configured according to _options_.
//      The system clock needs to be configured to run at 24 MHz. 
//
//  ARGUMENTS:
//      UINT16 baudRate
//          UART_BAUDRATE_4800
//          UART_BAUDRATE_9600
//          UART_BAUDRATE_19200
//          UART_BAUDRATE_38400
//          UART_BAUDRATE_57600
//          UART_BAUDRATE_115200
//      UINT16 clkFreq
//          Device clock frequency in kHz
//      UINT8 options
//          DEFAULT_MODE
//          UART_ISR
//------------------------------------------------------------------------------------------------------
void halUartSetup(UINT8 baudRate, UINT8 options) {
    switch (baudRate) {
        
        case UART_BAUDRATE_4800:
            TH1 = 48;
            break;

        case UART_BAUDRATE_9600:
            TH1 = 152;
            break;
            
        case UART_BAUDRATE_19200:
            TH1 = 204;
            break;

        case UART_BAUDRATE_38400:
            TH1 = 178;
            CKCON |= BM_SCA0;
            break;
            
        case UART_BAUDRATE_57600:
            TH1 = 48; 
            CKCON |= BM_T1M;
            break;

        case UART_BAUDRATE_115200:  
            TH1 = 152;
            CKCON |= BM_T1M;
            break;
    }
    TIMER1_RUN(TRUE); 
    TMOD |= BM_T1M1;                //8-bit counter/timer with auto-reload
    SCON0 = (options & 0xF0);

    // The TI bit is used to indicate that the previous transmission has completed and it's okay to write to 
    // SBUF0 again.
    if ((options & 0x0F) == UART_ISR) {
        INT_ENABLE(INUM_UART0, INT_ON);
        ENABLE_GLOBAL_INT(TRUE);
    } else {
        INT_SETFLAG(INUM_UART0_TX, INT_SET);
    }
}// halUartSetup




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: UartSetup.c,v $
 * Revision 1.4  2007/06/29 14:00:19  a0190590
 * Aligned control of UART interrupt flag (for polling mode) with initial revision.
 * Revision 1.3 provided that the standard putchar(...) function be customized,
 * which is not applied for all software examples yet.
 *
 * Revision 1.3  2006/06/29 12:56:12  a0190590
 * Corrected interrupt flag control according to modified/custom putchar(...) function.
 *
 * Revision 1.2  2005/02/07 14:46:04  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/

