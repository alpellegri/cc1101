/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains a test program for showing how to use the different     *
 *      ***   +             SPI library function toward the CC2500.The program will also use the UART  *
 *      ***   + +   ***     to display register values on a PC running hyper terminal.                 *
 *      ***   +++   ***     Spi.c                                                                      *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:        See end of file                                                            *
 ******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>
#include <Spi.h>
#include <stdio.h>
#include <ctype.h>




//-------------------------------------------------------------------------------------------------------
//  Global Variables

BYTE idata buffer[8];
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      The purpose of this program is to show the use of the different SPI function supported in the
//      hal library.  
//      on a PC running hyper terminal.
//      The port settings should be as follows:
//
//      Bits per second:    115200
//      Data bits:          8
//      Parity:             None
//      Stop bits:          1
//      Flow control:       None
//-------------------------------------------------------------------------------------------------------
void main (void) {
    UINT8 menuEntry;
    
    #ifdef STAND_ALONE
    
		// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
    	// SYSCLK = 4X Clock Multiplier / 2     
    	CLOCK_INIT();
    #endif

	// Set up the crossbar and I/O ports to communicate with the SmartRF04EB peripherals
    IO_PORT_INIT();

    // Initialize the LCD display. The SMBus uses timer 0 to generate SCL
    ebLcdInit();

    ebLcdUpdate("Spi.c", NULL);

    // 8-bit Uart with variable baudrate, logic level of stop bit is ignored. User must poll the 
    // RI_0 and TI_0 flag to determine when a byte arrives or when the TX buffer is empty. 
    // Timer1 is used as a baudrate generator
    halUartSetup(UART_BAUDRATE_115200, DEFAULT_MODE);

    SPI_INIT(SCLK_6_MHZ);

    POWER_UP_RESET_CCxxx0();

    displayMenu();

    // Enable the UART receiver
    UART_RX_ENABLE();

    // Infinite loop
    while (TRUE) {        
        UART_WAIT_AND_RECEIVE(menuEntry);
        printf("\n\n\r");
        handleMenu(menuEntry);
    }
}




//------------------------------------------------------------------------------------------------------
//  void displayMenu(void)
//
//  DESCRIPTION:
//      Function which displays the Menu on the PC    
//------------------------------------------------------------------------------------------------------
void displayMenu(void) {
    printf ("\n\rCCxx00 Menu:\n\n\r");
    printf ("R: Reset CCxx00\n\r");
    printf ("W: Write Register Settings to CCxx00\n\r");
    printf ("V: Display version number\n\r");
    printf ("D: Display All Registers\n\r");
    printf ("P: Write to PATABLE\n\r");
    printf ("L: Read PATABLE\n\r");
}




//------------------------------------------------------------------------------------------------------
//  void handleMenu(UINT8 menuEntry)
//
//  DESCRIPTION:
//      This will perform some SPI related code based on what the user enters in hyperterminal.          
//------------------------------------------------------------------------------------------------------
void handleMenu(UINT8 menuEntry) {
    UINT8 i;
    UINT8 regCounter;

    switch (toupper(menuEntry)) {
        
        case 'R':
            RESET_CCxxx0();
            displayMenu();    
            break;
    
        case 'W':
            
            // Write register settings
            halRfWriteRfSettings(&rfSettings);
            displayMenu();
            break;

        case 'D':
            
            // Write all registers to the hyper terminal
            for(regCounter = 0x00; regCounter <= 0x2E; regCounter++) {
                printf ("Addr. 0x%02X : ", (INT16)regCounter);
                printf ("0x%02X\n\r", (INT16)halSpiReadReg(regCounter));
            }
            displayMenu();
            break;

        case 'V':
            printf ("\n\rVersion: 0x%02X\n\r",(INT16)halSpiReadStatus(CCxxx0_VERSION));
            displayMenu();
            break;

        case 'P':

            // Write to PATABLE
            halSpiWriteBurstReg(CCxxx0_PATABLE, paTable, sizeof(paTable));
            displayMenu();
            break;

        case 'L':

            // Read PATABLE
            halSpiReadBurstReg(CCxxx0_PATABLE, buffer, sizeof(paTable));
            for(i = 0; i < 8; i++)
                printf ("PATABLE[%bu]: 0x%02X \n\r",i, (INT16)buffer[i]);
            displayMenu();
            break;

        default:
            break;
    }
}




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Spi.c,v $
 * Revision 1.10  2006/04/25 14:49:48  a0190596
 * added POWER_UP_RESET_CCxxx0()
 * halSpiWriteBurstReg(CCxxx0_PATABLE, paTable, sizeof(paTable));
 * replaced by
 * halSpiWriteReg(CCxxx0_PATABLE, paTable);
 *
 * Revision 1.9  2006/03/31 13:28:13  a0190596
 * POWER_UP_RESET_CCxxx0() removed
 *
 * Revision 1.8  2006/03/17 10:10:07  a0190596
 * New file structure might have caused variables, defines, and functions to move.
 *
 * Revision 1.7  2005/10/25 12:18:02  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 * Register settings moved to separate *.h file
 *
 * Revision 1.6  2005/07/11 12:14:13  sna
 * no message
 *
 * Revision 1.5  2005/06/14 11:08:51  sna
 * no message
 *
 * Revision 1.4  2005/04/11 13:31:36  sna
 * Updated the RF_SETTINGS struct according to SmartRF Studio 6.0.4
 *
 * Revision 1.3  2005/04/11 11:59:46  sna
 *  Register settings are changed and comments are added
 *  PATABLE is changed
 *
 * Revision 1.2  2005/02/07 14:31:18  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
