/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains a loop-back test for the audio part of the SmartRF04EB  *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Audio.c                                                                    *
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
//      In this example, the PCA is configured as an 8-bit PWM. The audio input is sampled every 10 us,
//      and the ADC value is used to adjust the duty cycle of the PWM. A microphone and a headset should
//      be connected to the SmartRF04EB when running this code.
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

    ebLcdUpdate("Audio.c", NULL);

    // Initialize the ADC converter
    ebAdcInit(ADC_AUDIO);

    // Period
    SET_RELOAD_VALUE_TIMER1(10, 24000);

    // Timer1, clock source is SYSCLK, 8-bit counter/timer with auto-reload, interrupt enabled
    halSetupTimer01(TIMER_1, SYSCLK_DIV_1, MODE_2, INT_ON);

    // Setup PCA (Programmable Counter Array)
    PCA0MD |= BM_CPS2;              // PCA counter timebase = SYSCLK
    PCA0CPM0 = BM_ECOM | BM_PWM;    // comparator function and 8-bit PWM mode enable
    CR = 1;                         //Enable PCA

    // Infinite loop
    while (TRUE);
}




//-------------------------------------------------------------------------------------------------------
//  void TIMER1_ISR(void)
//
//  DESCRIPTION:
//      Everytime a Timer1 interrupt occurs, the ADC is sampled and the duty cycle of the
//              PWM is adjusted
//-------------------------------------------------------------------------------------------------------
void TIMER1_ISR(void) interrupt INUM_TIMER1 {
   
    // Sample the ADC
    ADC_SAMPLE();

    // Adjust the duty cycle
    PCA0CPH0 = ADC0H;      
}




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Audio.c,v $
 * Revision 1.3  2005/10/25 11:54:27  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 *
 * Revision 1.2  2005/02/07 14:18:40  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/

