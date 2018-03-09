/***********************************************************************************
    Filename: hal_joystick.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_JOYSTICK_H
#define HAL_JOYSTICK_H

#ifdef __cplusplus
extern "C" {
#endif

enum {
    HAL_JOYSTICK_CENTER,
    HAL_JOYSTICK_LEFT,
    HAL_JOYSTICK_RIGHT,
    HAL_JOYSTICK_UP,
    HAL_JOYSTICK_DOWN
};

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void  halJoystickInit(void);
uint8 halJoystickGetPosition(void);
bool  halJoystickPushed(void);


#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif
