/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains a test program for showing how to use the joystick and  *
 *      ***   +             the LCD display. The display will show the joystick position.              *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Joystick.c                                                                 *
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
//  void main(void)
//
//  DESCRIPTION:
//      This program reads the ADC and thus acquire the joystick position. The position of the joystick 
//      is displayed on the LCD.
//      
//      |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//      | J | o | y | s | t | i | c | k | . | c |   |   |   |   |   |   |
//      |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//      | P | o | s | i | t | i | o | n | : |   | C | e | n | t | e | r |
//      |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//-------------------------------------------------------------------------------------------------------
void main (void) {
    UINT8 joystickPosition;

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

    // Infinite loop
    while (TRUE) {

        // Get current position of joystick
        joystickPosition = ebGetJoystickPosition();

        switch (joystickPosition) {
        
            case JOYSTICK_UP:
                ebLcdUpdate("Joystick.c", "Position: Up");
                break;
    
            case JOYSTICK_DOWN:
                ebLcdUpdate("Joystick.c", "Position: Down");
                break;

            case JOYSTICK_LEFT:
                ebLcdUpdate("Joystick.c", "Position: Left");
                break;

            case JOYSTICK_RIGHT:
                ebLcdUpdate("Joystick.c", "Position: Right");
                break;

            case JOYSTICK_PUSHED:
                ebLcdUpdate("Joystick.c", "Position: Pushed");
                break;

            default:
                ebLcdUpdate("Joystick.c", "Position: Center");
                break;
        }
    }
}




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Joystick.c,v $
 * Revision 1.3  2005/10/25 11:55:44  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 *
 * Revision 1.2  2005/02/07 14:23:35  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
