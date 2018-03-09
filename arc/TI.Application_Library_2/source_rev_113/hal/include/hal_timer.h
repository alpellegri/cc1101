/***********************************************************************************
    Filename: hal_timer.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_TIMER_H
#define HAL_TIMER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_types.h>

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void halTimerInit(uint16 rate);
void halTimerRestart(void);
void halTimerIntConnect(ISR_FUNC_PTR isr);
void halTimerIntEnable(void);
void halTimerIntDisable(void);


#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif

