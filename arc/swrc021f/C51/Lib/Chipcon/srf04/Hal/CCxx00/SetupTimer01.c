/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements a function for setting up                             *
 *      ***   +             timer 0 or timer 1.                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SetupTimer01.c                                                             *
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
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>




//-------------------------------------------------------------------------------------------------------
#define TIMER0_SYSCLK()             (CKCON |= 0x04)
#define TIMER0_PRESCALER()          (CKCON &= (~0x04))
#define TIMER1_SYSCLK()             (CKCON |= 0x08)
#define TIMER1_PRESCALER()          (CKCON &= (~0x08))
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// void halSetupTimer01(UINT8 timer01, UINT8 clkSource, UINT8 mode, BOOL timerInt)
//
//  DESCRIPTION:
//      Function used to set up timer 0 or timer 1. Be aware that Timer 0 and Timer 1 share the same
//      prescaler. The function enables the given timer. Allowing the timer to be controlled by the 
//      external input signal can be done manually by setting GATE0 or GATE1 to ‘1'.
//      If mode 2 is chosen, the SET_RELOAD_VALUE_TIMER0 or  SET_RELOAD_VALUE_TIMER1 macro has to
//      be called before this function.
//
//  ARGUMENTS:
//      UINT8 timer01
//          TIMER_0
//          TIMER_1
//
//      UINT8 clkSource 
//          SYSCLK_DIV_12
//          SYSCLK_DIV_4
//          SYSCLK_DIV_48
//          EXTCLK_DIV_8
//          SYSCLK_DIV_1
//
//      UINT8 mode
//          MODE_0  (13-bit counter/timer)
//          MODE_1  (16-bit counter/timer)
//          MODE_2  (8-bit counter/timer with auto-reload)
//
//          IMPORTANT NOTICE:
//          Timer 0 has a fourth mode (Mode 3). This mode has some restrictions and needs to be  
//          configured manually 
//      
//      BOOL timerInt
//          TRUE: Enable timer interrupt
//          False: Do nothing (does not disable timer interrupt if it has been enabled outside this
//          function).
//-------------------------------------------------------------------------------------------------------
void halSetupTimer01(UINT8 timer01, UINT8 clkSource, UINT8 mode, BOOL timerInt) {
    
    UINT8 temp = 0xFC;  

    // Set mode
    if (timer01 == TIMER_0) {
        TMOD &= 0xFC;
        TMOD |= mode;
    }
    else if (timer01 == TIMER_1) {
        TMOD &= 0xCF;
        TMOD |= (mode << 4);
    }

    // Timer 0 
    if (timer01 == TIMER_0) {
        if (clkSource > 3)
            TIMER0_SYSCLK();
        else {
            TIMER0_PRESCALER();
            
            //Set prescaler
            temp &= CKCON;
            CKCON = temp | clkSource;
        }
            
        // Enable Timer 0 interrupt ?
        if (timerInt) {
            INT_ENABLE(INUM_TIMER0, INT_ON);
            ENABLE_GLOBAL_INT(INT_ON);
        }
        TIMER0_RUN(TRUE);
    }

    // Timer 1
    else if (timer01 == TIMER_1) {
        if (clkSource > 3)
            TIMER1_SYSCLK();
        else {
            TIMER1_PRESCALER();
            
            //Set prescaler
            temp &= CKCON;
            CKCON = temp | clkSource;
        }
        // Enable Timer 1 interrupt ?
        if (timerInt) {
            INT_ENABLE(INUM_TIMER1, INT_ON);
            ENABLE_GLOBAL_INT(INT_ON);
        }
        TIMER1_RUN(TRUE);
    }
}// halSetupTimer01




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SetupTimer01.c,v $
 * Revision 1.3  2005/07/11 12:28:46  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:45:10  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
