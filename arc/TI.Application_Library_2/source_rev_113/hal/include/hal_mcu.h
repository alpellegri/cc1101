/***********************************************************************************
    Filename: hal_mcu.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_MCU_H
#define HAL_MCU_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_types.h>

enum {
    HAL_MCU_LPM_0,
    HAL_MCU_LPM_1,
    HAL_MCU_LPM_2,
    HAL_MCU_LPM_3,
    HAL_MCU_LPM_4
};




//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void halMcuInit(void);
void halMcuWaitUs(uint16 usec);
void halMcuSetLowPowerMode(uint8 mode);


#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif
