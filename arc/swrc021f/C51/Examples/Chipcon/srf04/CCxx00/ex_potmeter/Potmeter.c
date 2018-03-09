/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains a test program for showing how to read the potmeter     *
 *      ***   +             and display the value on the LCD display. The potmeter is                  *
 *      ***   + +   ***     read everytime the button S1 is pushed.                                    *
 *      ***   +++   ***     Potmeter.c                                                                 *
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




//-------------------------------------------------------------------------------------------------------
//  Global Variables
BYTE asciiString[4];
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Function declarations
void intToAscii(UINT8 value);
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This program reads the ADC and thus acquire the potmeter value. The potmeter value is displayed 
//      on the LCD display every time S1 is being pushed.
//-------------------------------------------------------------------------------------------------------
void main (void) {

    #ifdef STAND_ALONE
    
		// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
    	// SYSCLK = 4X Clock Multiplier / 2     
    	CLOCK_INIT();
    #endif

	// Set up the crossbar and I/O ports to communicate with the SmartRF04EB peripherals
    IO_PORT_INIT();

    // Initialize the LCD display. The SMBus uses timer 0 to generate SCL
    ebLcdInit();

    ebLcdUpdate("Potmeter.c", "Press S1");

    // Initialize the ADC converter
    ebAdcInit(ADC_POT);

    // Infinite loop
    while (TRUE) {
        if (ebButtonPushed()) {
            intToAscii(ebReadPotentiometer());
            ebLcdUpdate("Potmeter Value", asciiString);
        }
    }
}




//-------------------------------------------------------------------------------------------------------
//  BYTE intToAscii(UINT16 value)
//
//  DESCRIPTION:
//              Takes a 16 bits integer as input and converts it to ascii
//
//      ARGUMENTS:
//              UINT16 value
//                      The value to be converted
//-------------------------------------------------------------------------------------------------------
void intToAscii(UINT8 value) {
    UINT8 i;
    UINT8 j = 0;
    UINT8 digit_start = 0;
    UINT16 digit = 0;
    UINT16 denom = 100;

    if (value == 0) {
        asciiString[0] = '0';
        asciiString[1] = NULL;
    } else {
        for(i = 3; i > 0; i--) {
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




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Potmeter.c,v $
 * Revision 1.4  2005/10/25 12:10:46  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 *
 * Revision 1.3  2005/07/11 12:13:17  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:30:20  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/

