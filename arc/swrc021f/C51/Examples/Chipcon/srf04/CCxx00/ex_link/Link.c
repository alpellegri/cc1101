/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This program lets the user test a simple rf link                           *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Link.c                                                                     *
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
#include <Link.h>





//-------------------------------------------------------------------------------------------------------
// DESCRIPTION:
//
//      By moving the joystick right and left, the user can set up one unit for TX (left) or RX (right).
//      After setting the correct mode, the user must press the joystick button to start the link tester.
//      The transmitter will send one packet every time the S1 button is pushed.
//      Number of packets that have been transmitted is showed on the LCD display. The RX unit will display
//      number of packets received with CRC OK. 
//      The program uses halRfSendPacket() and halRfReceivePacket()
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

BYTE xdata txBuffer[] = {20, 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19};

BYTE xdata rxBuffer[61];    // Length byte  + 2 status bytes are not stored in this buffer
UINT8 xdata mode = MODE_NOT_SET;

BYTE xdata asciiString[11];
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This program shows how how to set up a link between two units.      
//----------------------------------------------------------------------------------------------------
void main (void) {

    UINT8 xdata joystickPosition = JOYSTICK_CENTER;
    UINT8 xdata prevJoystickPosition = JOYSTICK_CENTER;
    UINT32 packetsReceived = 0;
    UINT32 packetsSent = 0;
    UINT8 length;

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

    halRfWriteRfSettings(&rfSettings);

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

            case TX:

                // Infinite loop    
                while (TRUE) {

                    while (!ebButtonPushed());

                    halRfSendPacket(txBuffer, sizeof(txBuffer));

                    intToAscii(++packetsSent);
                    ebLcdUpdate("Sent:", asciiString);
                }
                break;
    
            case RX:
        
                // Infinite loop    
                while (TRUE) {

                    length = sizeof(rxBuffer);
                    if (halRfReceivePacket(rxBuffer, &length)) {
                        intToAscii(++packetsReceived);
                        ebLcdUpdate("Received:", asciiString);
                    }
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
 * $Log: Link.c,v $
 * Revision 1.14  2006/04/25 14:53:11  a0190596
 * added POWER_UP_RESET_CCxxx0()
 *
 * Revision 1.13  2006/03/31 13:14:40  a0190596
 * POWER_UP_RESET_CCxxx0() removed
 *
 * Revision 1.12  2006/03/17 10:16:19  a0190596
 * New file structure might have caused variables, defines, and functions to move.
 *
 * Revision 1.11  2005/11/09 09:56:24  sna
 * no message
 *
 * Revision 1.10  2005/10/28 13:48:58  sna
 * no message
 *
 * Revision 1.9  2005/10/28 09:38:08  sna
 * no message
 *
 * Revision 1.8  2005/10/25 12:00:27  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 * Register settings moved to separate *.h file
 *
 * Revision 1.7  2005/07/11 12:06:01  sna
 * no message
 *
 * Revision 1.6  2005/06/14 11:07:40  sna
 * no message
 *
 * Revision 1.5  2005/05/04 11:20:42  sna
 * Removed Manual Calibration
 *
 * Revision 1.4  2005/04/11 13:31:00  sna
 * Updated the RF_SETTINGS struct according to SmartRF Studio 6.0.4
 *
 * Revision 1.3  2005/04/11 11:41:34  sna
 * #include <stdio.h> removed
 * #include <ctype.h> removed
 * Register settings are changed and comments are added
 * PATABLE is changed
 *
 * Revision 1.2  2005/02/07 14:29:22  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/







