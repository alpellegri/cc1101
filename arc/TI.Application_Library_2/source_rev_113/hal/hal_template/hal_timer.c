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


//----------------------------------------------------------------------------------
//  void halTimerInit(uint16 rate)
//
//  DESCRIPTION:
//    Set up the timer to generate an interrupt every "rate" microseconds.
//    Use halTimerIntConnect() to connect an ISR to the interrupt.
//----------------------------------------------------------------------------------
void halTimerInit(uint16 rate)
{
    // Implementation is target specific.
    // Select a clock that suits your need. An external low
    // frequency clock oscillator may be a good choice if you want to
    // set the MCU in a low power mode but still be able to use the timer.
}

//----------------------------------------------------------------------------------
//  void halTimerRestart(void)
//
//  DESCRIPTION:
//    Restart timer. The timer is first stopped, then restarted, counting up from 0
//----------------------------------------------------------------------------------
void halTimerRestart(void)
{
    // Restart timer. Implementation depends on the actual timer in use.
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
    // Enable interrupts. Depends on the actual timer in use.
}

//----------------------------------------------------------------------------------
//  void halTimerIntDisable(void)
//----------------------------------------------------------------------------------
void halTimerIntDisable(void)
{
    // Disable interrupts. Depends on the actual timer in use.
}

//----------------------------------------------------------------------------------
//  void timer_ISR(void)
//
//  DESCRIPTION:
//    ISR framework for the timer component
//----------------------------------------------------------------------------------
#pragma vector=TIMERnn_VECTOR  // Select the appropriate timer interrupt vector
__interrupt void timer_ISR(void)
{
    if (fptr != NULL)
    {
        (*fptr)();
    }
    __low_power_mode_off_on_exit();
}



