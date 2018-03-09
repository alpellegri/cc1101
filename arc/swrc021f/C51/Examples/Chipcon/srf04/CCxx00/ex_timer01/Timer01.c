/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains a test program for showing how to use timer 1 and the   *
 *      ***   +             the related library functions.                                             *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Timer01.c                                                                  *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                    Keil C51 V7.50                                                         *
 * Target platform:             Chipcon CCxxx0 (Silabs F320)                                           *
 * Author:                      SNA                                                                    *
 *******************************************************************************************************
 * Revision history:            See end of file                                                        *
 ******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>




//-------------------------------------------------------------------------------------------------------
//  Global Variables
UINT8 potmeterValue;        // Value used to set the high byte of the 16-bit Timer 1                        
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro used to show a 4 bits binary number using the 4 LEDs on the SmartRF04EB
// Green LED: LSB
// Blue LED: MSB 
#define SET_LED_MASK(n) \
    do { \
        if (n & 0x01) SET_GLED(LED_ON); else SET_GLED(LED_OFF); \
        if (n & 0x02) SET_RLED(LED_ON); else SET_RLED(LED_OFF); \
        if (n & 0x04) SET_YLED(LED_ON); else SET_YLED(LED_OFF); \
        if (n & 0x08) SET_BLED(LED_ON); else SET_BLED(LED_OFF); \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This program read the potmeter value and uses this 8 bit value to set the high byte of the 
//      16-bit Timer 1. Every time timer1 overflows, a 4 bit binary number is updated on the LEDs
//      (this is done in the timer1 ISR)
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
    
    ebLcdUpdate("Timer01.c", NULL);

    // Initialize the ADC converter
    ebAdcInit(ADC_POT);
    
    // Timer1, clock source is SYSCLK / 48, 16-bit counter/timer, interrupt enabled
    halSetupTimer01(TIMER_1, SYSCLK_DIV_48, MODE_1, INT_ON);

    // Infinite loop    
    while (TRUE) {
        ENABLE_GLOBAL_INT(INT_OFF);
        potmeterValue = ebReadPotentiometer();
        ENABLE_GLOBAL_INT(INT_ON);
    }
}




//-------------------------------------------------------------------------------------------------------
//  void TIMER1_ISR(void)
//
//  DESCRIPTION:
//      In this interrupt routine the LEDs are updated 
//-------------------------------------------------------------------------------------------------------
void TIMER1_ISR(void) interrupt INUM_TIMER1 {
    static UINT8 counter = 0;
   
 // Display the counter word on LEDs
    SET_LED_MASK(counter);
    counter++;
    if (counter == 15)
        counter = 0;
    TH1 = potmeterValue;
}




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Timer01.c,v $
 * Revision 1.4  2005/10/25 12:21:02  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 *
 * Revision 1.3  2005/07/11 12:14:49  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:32:38  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
