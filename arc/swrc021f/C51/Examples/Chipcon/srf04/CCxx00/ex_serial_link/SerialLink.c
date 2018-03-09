/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This program lets the user test a simple rf link using serial synchronous  *
 *      ***   +             mode.                                                                      *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SerialLink.c                                                               *
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
#include <Chipcon\srf04\culsrf04.h>
#include <SerialLink.h>
#include <stdio.h>
#include <ctype.h>





//-------------------------------------------------------------------------------------------------------
// DESCRIPTION:
//
//      By moving the joystick right and left, the user can set up one unit for TX (left) or RX (right).
//      After setting the correct mode, the user must press the joystick button to start the link tester.
//      The transmitter will send one packet every time the S1 button is pushed.
//      Number of packets that have been transmitted is showed on the LCD display. The RX unit will display
//      number of packets received with CRC OK. 
//      The program uses halRfSendPacketSerial() and halRfReceivePacketSerial()
//
//      |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//      | S | e | n | t | : |   |   |   |   |   |   |   |   |   |   |   |
//      |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//      | 2 | 5 | 8 |   |   |   |   |   |   |   |   |   |   |   |   |   |
//      |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  Global Variables


UINT16 xdata checksum;
UINT8 xdata receivedCRC[2];

// Remember to make room for CRC in the packet if CRC check is enabled 
BYTE xdata txBuffer[] = {0x55,  0x55,   0x55,   0x55,   // Preamble
                         SYNC3, SYNC2,  SYNC1,  SYNC0,  // Sync word
                         3,     'A',    'B',    'C',    // Payload
                         CRC1, CRC0};                   // CRC (optional)


BYTE xdata rxBuffer[20];
UINT8 xdata mode = MODE_NOT_SET;

BYTE xdata asciiString[11];
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This program shows how how to set up a link between two units.      
//-------------------------------------------------------------------------------------------------------
void main (void) {

    UINT8 xdata joystickPosition = JOYSTICK_CENTER;
    UINT8 xdata prevJoystickPosition = JOYSTICK_CENTER;
    UINT32 xdata packetsReceived = 0;
    UINT32 xdata packetsSent = 0;
    UINT8 data i = 0;
	UINT8 data j = 0;
    UINT8 data dataByte = 0;

    #ifdef STAND_ALONE
    
		// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
    	// SYSCLK = 4X Clock Multiplier / 2     
     	CLOCK_INIT();
    #endif

	// Set up the crossbar and I/O ports to communicate with the SmartRF04EB peripherals
    IO_PORT_INIT();

    // Initialize the LCD display. The SMBus uses timer 0 to generate SCL
    ebLcdInit();

    ebLcdUpdate("Select Mode", "Using Joystick");

    // Initialize the ADC converter
    ebAdcInit(ADC_JOY);

    SPI_INIT(SCLK_6_MHZ);

    POWER_UP_RESET_CCxxx0();

    // 8-bit Uart with variable baudrate, logic level of stop bit is ignored. User must poll the 
    // RI_0 and TI_0 flag to determine when a byte arrives or when the TX buffer is empty. 
    // Timer1 is used as a baudrate generator
    halUartSetup(UART_BAUDRATE_115200, DEFAULT_MODE);

    halRfWriteRfSettings(&rfSettings);
    halSpiWriteReg(CCxxx0_IOCFG1, 0x0B);    // Serial clock
    
    halSpiWriteReg(CCxxx0_PATABLE, paTable);
	
    // Select Tx or Rx mode by moving the joystick right or left
    do {
        
        halWait(250);
        
        // Get current position of joystick
        joystickPosition = ebGetJoystickPosition();

        if (prevJoystickPosition != joystickPosition)
            parseMenu(joystickPosition);

        prevJoystickPosition = joystickPosition;
               
    } while (!ebJoyPushed());
    
    // Infinite loop    
    while (TRUE) {
        switch (mode) {

            case RX:
        
                // Infinite loop    
                while (TRUE) {
                    if (halRfReceivePacketSerial(rxBuffer, SYNC3, SYNC2, SYNC1, SYNC0, VARIABLE_LENGTH, CRC_ENABLE)) {
                        intToAscii(++packetsReceived);
                        ebLcdUpdate("Received:", asciiString);
                    }
                }
                break;
    
            case TX:

                P0MDOUT |= GDO0_;   // In TX, GDO0 is serial data input, and must be configured as a push/pull output
                                    // on the MCU
                while (TRUE) {

                    while (!ebButtonPushed());
                
                    halRfSendPacketSerial(txBuffer, sizeof(txBuffer), START_OF_PAYLOAD, CRC_ENABLE);

                    intToAscii(++packetsSent);
                    ebLcdUpdate("Sent:", asciiString);
                }
                break;         
        }
    }   
}




//-------------------------------------------------------------------------------------------------------
//  void intToAscii(UINT32 value)
//
//  DESCRIPTION:
//		Takes a 32 bits interger as input and converts it to ascii. Puts the result in the global
//      variable asciiString[]
//
//	ARGUMENTS:
//		UINT32 value
//			The value to be converted
//-------------------------------------------------------------------------------------------------------
void intToAscii(UINT32 value) {
    UINT8 i;
    UINT8 j = 0;
    UINT8 digit_start = 0;
    UINT16 digit = 0;
    UINT32 denom = 1000000000;

    if (value == 0) {
        asciiString[0] = '0';
        asciiString[1] = NULL;
    } else {
        for(i = 10; i > 0; i--) {
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
//  void parseMenu(UINT8 joystickPosition) 
//
//  DESCRIPTION: 
//    
//-------------------------------------------------------------------------------------------------------
void parseMenu(UINT8 joystickPosition) {
	
	switch (joystickPosition) {

		case JOYSTICK_LEFT:
			if (mode == RX || mode == MODE_NOT_SET) {
                mode = TX;
                ebLcdUpdate("Mode: Tx", NULL);
            }
			break;

		case JOYSTICK_RIGHT:
			if (mode == TX || mode == MODE_NOT_SET) {
                mode = RX;
                ebLcdUpdate("Mode: Rx", NULL);
            }
			break;

		default:
			break;
	}
}// parseMenu




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: SerialLink.c,v $
 * Revision 1.5  2006/04/25 14:59:07  a0190596
 * added POWER_UP_RESET_CCxxx0()
 *
 * Revision 1.4  2006/03/31 13:27:29  a0190596
 * POWER_UP_RESET_CCxxx0() removed
 *
 * Revision 1.3  2006/03/17 10:24:58  a0190596
 * New file structure might have caused variables, defines, and functions to move.
 *
 * Revision 1.2  2005/12/02 12:05:53  sna
 * Changed Port 0 configuration on the MCU
 * to have the code running at SmartRF04EB 1.9
 *
 * Revision 1.1  2005/11/09 09:59:15  sna
 * Initial version in CVS.
 *
 *
 *
 ******************************************************************************************************/









