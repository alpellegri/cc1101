/***********************************************************************************
    Filename: hal_timer.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include "hal_types.h"
#include "hal_defs.h"
#include "hal_timer.h"
#include "hal_board.h"
#include "hal_int.h"


static ISR_FUNC_PTR fptr;
static uint16 mode;


//----------------------------------------------------------------------------------
//  void halTimerInit(uint16 rate)
//
//  DESCRIPTION:
//    Set up the timer to generate an interrupt every "rate" microseconds.
//    Use halTimerIntConnect() to connect an ISR to the interrupt.
//----------------------------------------------------------------------------------
void halTimerInit(uint16 rate)
{
    uint16 clock_divider = ID_0; // Default - don't divide input clock

    // rate in usec (time between timer interrupt)
    // The timer input clock is SMCLK @ 4 MHz
    // For 1 usec, the timer must count to 4 (approx)

    if (rate >= 16384)
        clock_divider = ID_2; // divide input clock with 4
    else
        rate <<= 2;            // multiply rate by 4

    // Set compare value
    TACCR0 = rate;

    // Compare mode, clear interrupt pending flag, disable interrupt
    TACCTL0 = 0;

    // Timer source SMCLK
    // Use calculated divider
    // Count up to TACCR0
    // Clear timer
    mode = TASSEL_2 | clock_divider | MC_1 | TACLR;
    TACTL = mode;
}

//----------------------------------------------------------------------------------
//  void halTimerRestart(void)
//
//  DESCRIPTION:
//    Restart timer. The timer is first stopped, then restarted, counting up from 0
//----------------------------------------------------------------------------------
void halTimerRestart(void)
{
    TACTL = 0;
    // Avoid compiler optimization (skipping the line above)
    asm(" nop");
    TACTL = mode;
}


//----------------------------------------------------------------------------------
//  void halTimerIntConnect(ISR_FUNC_PTR isr)
//----------------------------------------------------------------------------------
void halTimerIntConnect(ISR_FUNC_PTR isr)
{
    istate_t key;
    HAL_INT_LOCK(key);
    fptr = isr;
    HAL_INT_UNLOCK(key);
}

//----------------------------------------------------------------------------------
//  void halTimerIntEnable(void)
//----------------------------------------------------------------------------------
void halTimerIntEnable(void)
{
    TACCTL0 |= CCIE;
}

//----------------------------------------------------------------------------------
//  void halTimerIntDisable(void)
//----------------------------------------------------------------------------------
void halTimerIntDisable(void)
{
    TACCTL0 &= ~CCIE;
}

//----------------------------------------------------------------------------------
//  void timera0_ISR(void)
//
//  DESCRIPTION:
//    ISR framework for the timer component
//----------------------------------------------------------------------------------
#pragma vector=TIMERA0_VECTOR
__interrupt void timera0_ISR(void)
{
    if (fptr != NULL)
    {
        (*fptr)();
    }
    __low_power_mode_off_on_exit();
}



