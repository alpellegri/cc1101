
/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains a test program for showing how to use timer 2 and the   *
 *      ***   +             the related library functions.                                             *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Timer23.c                                                                  *
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
UINT16 countH = 0;    // Counter used in the timer 2 ISR
UINT16 countL = 0;    // Counter used in the timer 2 ISR                      
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This program sets up timer2 as two 8-bit timers.
//      TMR2H has a period of 10.6 us and TMR2L has a period of 128 us
//      In the timer2 ISR a LED is toggled every 10000th time the interrupt occurs.
//      High byte overflow: Toggle green LED
//      Low byte overflow:  Toggle red LED
//      
//      
//        |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|
//  GLED  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |    
//      --|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |--|  |  
//
//
//        |-----|     |-----|     |-----|     |-----|     |-----|     |-----|     |-----|     |-----|
//  RLED  |     |     |     |     |     |     |     |     |     |     |     |     |     |     |     |    
//      --|     |-----|     |-----|     |-----|     |-----|     |-----|     |-----|     |-----|     |--- 
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
    
    ebLcdUpdate("Timer23.c", NULL);

    // TMR2H: 24 MHz clk. source, 10.6 us period
    // TMR2L: 2 MHz clk. source, 21.2 us period
    SET_RELOAD_VALUE_TIMER2_8BIT(10.6, 21.2, 24000, 2000);

    // Timer2, clock source is SYSCLK and SYSCLK / 12, 2 8-bit timers with auto-reload, interrupt enabled
    halSetupTimer23(TIMER_2, SYSCLK_DIV_1, SYSCLK_DIV_12, MODE_1, INT_ON);

    // Infinite loop    
    while (TRUE);
}




//-------------------------------------------------------------------------------------------------------
//  void TIMER2_ISR(void)
//
//  DESCRIPTION:
//      This ISR checks what caused the interrupt (high byte or low byte  overflow) and clears the
//      corresponding interrupt-pending flag. Every 10000th interrupt, a LED is toggled. 
//-------------------------------------------------------------------------------------------------------
void TIMER2_ISR(void) interrupt INUM_TIMER2 {
    if (INT_GETFLAG(INUM_TIMER2_H)) {               // High byte overflow
        INT_SETFLAG(INUM_TIMER2_H, INT_CLR);
        if (countH++ == 10000) {
            countH = 0;
            P_LED1 = ~P_LED1;                       // Toggle green LED
        }
    } else {                                        // Low byte overflow
        INT_SETFLAG(INUM_TIMER2_L, INT_CLR);
        if (countL++ == 10000) {
            countL = 0;
            P_LED2 = ~P_LED2;                       // Toggle red LED
        }
    }
}




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Timer23.c,v $
 * Revision 1.3  2005/10/25 12:22:00  sna
 * Moved IO_PORT_INIT(); outside #ifdef
 *
 * Revision 1.2  2005/02/07 14:33:33  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
