/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements a function for setting up                             *
 *      ***   +             timer 2 or timer 3.                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SetupTimer23.c                                                             *
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
#define TIMER2_SYSCLK()             (CKCON |= 0x10)
#define TIMER2_EXT_CLK_SELECT()     (CKCON &= (~0x10))
#define TIMER2H_SYSCLK()            (CKCON |= 0x20)
#define TIMER2H_EXT_CLK_SELECT()    (CKCON &= (~0x20))
#define TIMER3_SYSCLK()             (CKCON |= 0x40)
#define TIMER3_EXT_CLK_SELECT()     (CKCON &= (~0x40))
#define TIMER3H_SYSCLK()            (CKCON |= 0x80)
#define TIMER3H_EXT_CLK_SELECT()    (CKCON &= (~0x80))
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#define SET_CLK_SOURCE_TIMER2(clkSourceH, clkSourceL, mode) \
    do { \
        if (clkSourceL == 4) \
            TIMER2_SYSCLK(); \
        else { \
            TIMER2_EXT_CLK_SELECT(); \
            if (clkSourceL == 0) \
                T2XCLK = 0; \
            else if (clkSourceL == 3) \
                T2XCLK = 1; \
        } \
        if (mode == 1) { \
            if (clkSourceH == 4) \
                TIMER2H_SYSCLK(); \
            else { \
                TIMER2H_EXT_CLK_SELECT(); \
                if (clkSourceH == 0) \
                    T2XCLK = 0; \
                else if (clkSourceH == 3) \
                    T2XCLK = 1; \
            } \
        } \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#define SET_CLK_SOURCE_TIMER3(clkSourceH, clkSourceL, mode) \
    do { \
        if (clkSourceL == 4) \
            TIMER3_SYSCLK(); \
        else { \
            TIMER3_EXT_CLK_SELECT(); \
            if (clkSourceL == 0) \
                TMR3CN &= 0xFE; \
            else if (clkSourceL == 3) \
                TMR3CN |= 0x01; \
        } \
        if (mode == 1) { \
            if (clkSourceH == 4) \
                TIMER3H_SYSCLK(); \
            else { \
                TIMER3H_EXT_CLK_SELECT(); \
                if (clkSourceH == 0) \
                    TMR3CN &= 0xFE; \
                else if (clkSourceH == 3) \
                    TMR3CN |= 0x01; \
            } \
        } \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void halSetupTimer23(UINT8 timerOption, clkSourceH, UINT8 clkSourceL, UINT8 mode, BOOL timerInt)
//
//  DESCRIPTION:
//      Function used to set up timer 2 or timer 3. SET_RELOAD_VALUE_TIMERx_16BIT(period, clock_kHz),
//      used for mode 0, or SET_RELOAD_VALUE_TIMERx_8BIT(periodH, periodL, clock_kHzH, clock_kHzL),
//      used for mode 1, should first be called (x = 2 or 3). The function enables the given timer. 
//      This function only support mode 0 and mode 1.
//
//  ARGUMENTS:
//      UINT8 timer23
//          TIMER_2
//          TIMER_3
//
//      UINT8 clkSourceH, clkSourceL 
//          SYSCLK_DIV_12
//          EXTCLK_DIV_8
//          SYSCLK_DIV_1
//
//          IMPORTANT NOTICE:
//          If mode 0 is chosen, clkSourceL should be initialized and clkSourceH is DON'T CARE
//          In mode 2, both clock sources must be given a value from the list above
//
//      UINT8 mode
//          MODE_0  (16-bit timer with auto-reload)
//          MODE_1  (8-bit timers with auto-reload)
//      
//      BOOL timerInt
//          TRUE: Enable timer interrupt
//          False: Do nothing (does not disable timer interrupt if it has been enabled outside this
//          function). 
//          For mode 0, only TMRxH will generate interrupt.         
//          For mode 1, both TMRxH AND TMRxL will generate interrupt. If only TMRxH shall generate 
//          interrupt, TFxLEN must be cleared manually after this function has been called 
//          IMPORTANT NOTICE:
//          TF2H and TF2L must be cleared by software in the ISR
//          Software must also check what caused the interrupt (high byte or low byte overflow)
//-------------------------------------------------------------------------------------------------------
void halSetupTimer23(UINT8 timer23, UINT8 clkSourceH, UINT8 clkSourceL, UINT8 mode, BOOL timerInt) {    
    
    // Set mode
    if (timer23 == TIMER_2) {
        TMR2CN &= 0xE7;
        TMR2CN |= mode << 3;  
    }
    else if (timer23 == TIMER_3) {
        TMR3CN &= 0xE7;
        TMR3CN |= mode << 3;
    } 
  
    // Timer 2 
    if (timer23 == TIMER_2) {
        SET_CLK_SOURCE_TIMER2(clkSourceH, clkSourceL, mode);

        // Enable Timer 2 interrupt ?
        if (timerInt) {
            INT_ENABLE(INUM_TIMER2, INT_ON);        // High byte interrupt enabled
            ENABLE_GLOBAL_INT(INT_ON);
            if (mode == MODE_1)
                TMR2CN |= BM_TF2LEN;                // Low byte interrupt enabled
        }
        TIMER2_RUN(TRUE);
    }
    
    // Timer 3
    else if (timer23 == TIMER_3) {
        SET_CLK_SOURCE_TIMER3(clkSourceH, clkSourceL, mode);
            
        // Enable Timer 3 interrupt ?
        if (timerInt) {
            INT_ENABLE(INUM_TIMER3, INT_ON);        // High byte interrupt enabled
            ENABLE_GLOBAL_INT(INT_ON);
            if (mode == MODE_1)
                TMR3CN |= BM_TF3LEN;                // Low byte interrupt enabled
        }
        TIMER3_RUN(TRUE);
    }
}// halSetupTimer23




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SetupTimer23.c,v $
 * Revision 1.3  2005/07/11 12:28:53  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:45:17  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
