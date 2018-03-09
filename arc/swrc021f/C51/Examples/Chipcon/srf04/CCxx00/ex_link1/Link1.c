/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This example shows how to transmit and receive packets with packet length  *
 *      ***   +             up to 255 bytes. The status byte is polled every 200 us in a timer1 int.   *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Link1.c                                                                    *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:     See end of file                                                               *
 *******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>
#include <Link1.h>





//-------------------------------------------------------------------------------------------------------
// DESCRIPTION:
//      This program allows the user to set up a simple link between two SmartRf04EBs with 2 CCxx00 EM 
//      modules connected. The joystick is used to navigate through a menu, setting different parameters. 
//
//      Parameters          | Values 
//      ------------------------------------------------------
//      Packet Length       | 10, 30, 50, 70, ...., 230, 250
//      Number of Packets   | 100, 200, 300, ......, 900, 1000
//      Whitening           | Enabled, Disabled
//      Radio Mode          | Rx, Tx
//
//      RX UNIT: 
//                  * Enable/disable Whitening
//                  * Set radio mode to RX. 
//                  * Move joystick down until the message 
//                    "Press S1 to start" is showed on the LCD display  
//                  * Press S1
//                  
//                  The LCD display will show number of packets received with CRC OK
//
//      TX UNIT:
//                  * Set packet length and number of packets to transmit
//                  * Enable/disable Whitening (set to the same as on the RX unit)
//                  * Set radio mode to TX
//                  * Press S1 to Start
//
//                  The LCD will show number of packets transmitted
//
//                  After all the packets have been transmitted, S1 can be pressed again to transmit 
//                  or the joystick can be used to change packet length and number of packets 
//                  to transmit
//
//      IMPORTANT NOTICE!
//      Be aware that the status byte polling cannot fully be trusted 
//      (See the CC1100, CC1150, CC2500, or 2550 Errata Note). Link2 will show how packets longer than 
//      the FIFO can be sent/received without using any SPI polling.
//      To run this example, the following register settings must be used:
//          PKTCTRL1.APPEND_STATUS = 1
//          PKTCTRL0.CRC_EN = 1
//          MCSM2.RX_TIME = 7
//          MSCM1.RXOFF_MODE = MSCM1.TXOFF_MODE = 0
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Global Variables

BYTE xdata txBuffer[258];   // 1 length byte + 255 bytes payload + 2 status bytes
BYTE xdata rxBuffer[258];   // 1 length byte + 255 bytes payload + 2 status bytes

UINT16 xdata packetsReceived = 0;

UINT8 xdata joystickPosition = JOYSTICK_CENTER;
UINT8 xdata prevJoystickPosition = JOYSTICK_CENTER;

UINT8 state = SETUP;    // Variable used to hold the next state in the state machine in the main loop
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This function takes care of all the MCU initialization and radio configuration. The main loop
//      is implemented as a state machine and a state diagram can be found in the
//      CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual 
//-------------------------------------------------------------------------------------------------------
void main (void) {

    UINT16 xdata packetsSent = 0;
    
    #ifdef STAND_ALONE

    	// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
    	// SYSCLK = 4X Clock Multiplier / 2
    	CLOCK_INIT();
    #endif

	// Set up the crossbar and I/O ports to communicate with the SmartRF04EB peripherals
    IO_PORT_INIT();

    // Initialize the LCD display. The SMBus uses timer 0 to generate SCL
    ebLcdInit();

    // Initialize the ADC converter
    ebAdcInit(ADC_JOY);

    SPI_INIT(SCLK_6_MHZ);

    POWER_UP_RESET_CCxxx0();
   
    halRfWriteRfSettings(&rfSettings);

    halSpiWriteReg(CCxxx0_PATABLE, paTable);

    pktDataInit();

    // Setup Timer1 to overflow every 200 us
    //  1/24000000 * x = 200 us => x = 4800
    //  2^13 - 4800 = 3392 = 0110101000000b
    //  => TH1 = 106
    //  => TL1 = 0
    TH1 = 106;
    TL1 = 0;

    // Clock source: 24 MHz, Mode: 13-bits timer
    halSetupTimer01(TIMER_1, SYSCLK_DIV_1, MODE_0, INT_OFF);

    handleAndDisplayMenu();

    // Infinite loop
    while (TRUE) {
        switch (state) {

            case SETUP:
            
                // This wait is implemented to give the LCD display enought time to be update
                // between each iteration
                halWait(250);

                joystickPosition = ebGetJoystickPosition();

                if (prevJoystickPosition != joystickPosition)
                    parseMenu(joystickPosition);

                prevJoystickPosition = joystickPosition;

                if ((index == START) && (ebButtonPushed())) {
                    INT_ENABLE(INUM_TIMER1, INT_ON);
                    if (menuData.radioMode == RX) {
                        state = RX_START;
                        ebLcdUpdate("Waiting for", "data");
                    }
                    else
                        state = TX_START;
                }
                break;            
                
            case TX_START:

                // Create data packet
                createDataPacket();
                
                if (packetsSent++ < menuData.numberOfPackets) {

                    state = TX_WAIT;

                    // Transmit packet with delay of 500 ms (250 * 200 us = 500 ms). The delay is 
                    // implemented so that it will be possible to view the packet counter on the LCD 
                    // display
                    pktStartTx(250);
                }
                else {
                    halWait(5000);
                    ebLcdUpdate("Press S1", "to start");
                    packetsSent = 0;
                    state = SETUP;
                    INT_ENABLE(INUM_TIMER1, INT_OFF);
                }
                break;

            case TX_WAIT:

                if (!pktData.txInProgress) {
                    if (pktData.pktTransmitted) {
                        intToAscii(packetsSent);
                        ebLcdUpdate("Sent:", asciiString);
                        pktData.pktTransmitted = FALSE;
                        state = TX_START;
                        
                    } else if (pktData.txUnderflowFlag) {
                        pktData.txBytesLeft = 0;
                        pktData.txUnderflowFlag = FALSE;
                        state = TX_START;
                    }
                }
                break;

            case RX_START:
                state = RX_WAIT;
                
                //Receive packet (no timeout)
                pktStartRx(0);
               
                break;

            case RX_WAIT:
                if (!pktData.rxTimeoutReached) {

                    if (pktData.pktReceived) {

                        pktData.pktReceived = FALSE;

                        // Check CRC_OK
                        if (pktData.pRxBuffer[pktData.pRxBuffer[0] + 2] & CRC_OK) {
                            intToAscii(++packetsReceived);
                            ebLcdUpdate("Received:", asciiString);
                        }
                        state = RX_START;
                    }
                } else {
                    ebLcdUpdate("Press S1", "to start");
                    pktData.rxTimeoutReached = FALSE;
                    state = SETUP;
                }
                break;

            default:
                break;
        }
    }
}




//-------------------------------------------------------------------------------------------------------
//  void TIMER1_ISR(void)
//
//  DESCRIPTION:
//      This interrupt occurs every 200 us, when timer1 overflows. The timer regsisters are reloaded and
//      both packet handler functions are called (pktRxHandler and pktTxHandler).
//-------------------------------------------------------------------------------------------------------
void TIMER1_ISR(void) interrupt INUM_TIMER1 {

    // Reload timer1 registers to overflow every 200 us
    TH1 = 106;
    TL1 = 0;
    pktTxHandler();
    pktRxHandler();
} // TIMER1_ISR




//-------------------------------------------------------------------------------------------------------
//  void intToAscii(UINT16 value)
//
//  DESCRIPTION:
//		Takes a 16 bits integer as input and converts it to ascii. The result is put in a global
//      variable (UINT8 xdata asciiString[6])
//
//	ARGUMENTS:
//		UINT16 value
//			The value to be converted
//-------------------------------------------------------------------------------------------------------
void intToAscii(UINT16 value) {
    UINT8 i;
    UINT8 j = 0;
    UINT8 digit_start = 0;
    UINT16 digit = 0;
    UINT16 denom = 10000;

    if (value == 0) {
        asciiString[0] = '0';
        asciiString[1] = NULL;
    } else {
        for(i = 5; i > 0; i--) {
            digit = value / denom;
            if((digit_start == 1) || (digit != 0)) {
                digit_start = 1;
                value %= denom;
                asciiString[j++] = (digit + '0');
            }
            denom /= 10;
        }
        asciiString[j++] = NULL;
    }
}// intToAscii





//-------------------------------------------------------------------------------------------------------
//  void pktDataInit(void)
//
//  DESCRIPTION:
//      Function to initial the pktData struck.
//
//-------------------------------------------------------------------------------------------------------
void pktDataInit(void) {

    // Setup pointers to tx and rx buffer
    pktData.pTxBuffer = txBuffer;
    pktData.pRxBuffer = rxBuffer;

    pktData.txInProgress = FALSE;
    pktData.txStartupTimeoutActive = FALSE;
    pktData.rxTimeoutReached = FALSE;
    pktData.rxTimeoutActive = FALSE;
    pktData.pktTransmitted = FALSE;
    pktData.pktReceived = FALSE;
    pktData.rxBytesLeft = 0;
    pktData.rxBytesLeft = 0;
    pktData.lengthByteRead = FALSE;
    pktData.txUnderflowFlag = FALSE;
}// pktDataInit






/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: Link1.c,v $
 * Revision 1.12  2007/01/25 12:14:34  a0190596
 * no message
 *
 * Revision 1.11  2007/01/23 15:46:20  a0190596
 * TX_WAIT state in Link1.c has been changed to properly handle underflow of the TX FIFO.
 *
 * Revision 1.10  2006/11/14 13:37:16  a0190596
 * Removed FEC option
 *
 * Revision 1.9  2006/10/06 13:25:41  a0190596
 * no message
 *
 * Revision 1.8  2006/04/25 14:55:57  a0190596
 * added POWER_UP_RESET_CCxxx0()
 *
 * Revision 1.7  2006/03/31 13:15:41  a0190596
 * POWER_UP_RESET_CCxxx0() removed
 *
 * Revision 1.6  2006/03/17 10:19:39  a0190596
 * New file structure might have caused variables, defines, and functions to move.
 * Comments are added.
 *
 * Revision 1.5  2005/10/25 12:04:26  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 * Register settings moved to separate *.h file
 *
 * Revision 1.4  2005/07/11 12:11:04  sna
 * no message
 *
 * Revision 1.3  2005/06/14 11:08:00  sna
 * no message
 *
 * Revision 1.2  2005/05/04 11:22:20  sna
 * Removed Manual Calibration
 *
 * Revision 1.1  2005/04/20 12:41:36  sna
 * Initial version in CVS
 *
 *
 *
 ******************************************************************************************************/
