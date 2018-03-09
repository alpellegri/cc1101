/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements a function for reading the joystick position          *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     GetJoystickPosition.c                                                      *
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
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>




//------------------------------------------------------------------------------------------------------
// Macroes used by the ebGetJoystickPosition
#define JOY_LEFT(adc)       ((adc > 96) & (adc < 123))  
#define JOY_RIGHT(adc)      ((adc > 164) & (adc < 191)) 
#define JOY_UP(adc)         ((adc > 196) & (adc < 223)) 
#define JOY_DOWN(adc)       ((adc >= 0) & (adc < 61))   
#define JOY_CENTER(adc)     ((adc > 227) & (adc < 254))
//------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  UINT8 ebGetJoystickPosition(void) 
//
//  DESCRIPTION:
//      This function will read the ADC to determine the current joystick position. ebAdcInit(ADC_JOY)
//      must be called before this function can be called.
//
//      RETURN VALUE:
//          UINT8
//              JOYSTICK_CENTER  0
//              JOYSTICK_UP      1
//              JOYSTICK_DOWN    2
//              JOYSTICK_LEFT    3
//              JOYSTICK_RIGHT   4              
//              JOYSTICK_PUSHED  5
//-------------------------------------------------------------------------------------------------------
UINT8 ebGetJoystickPosition(void) {
    UINT8 adc;
    
    // Sample the ADC
    ADC_SAMPLE();

    adc = ADC0H;
    
    if (JOY_LEFT(adc))
        return JOYSTICK_LEFT;
    else if (JOY_RIGHT(adc))
        return JOYSTICK_RIGHT;
    else if (JOY_UP(adc))
        return JOYSTICK_UP;
    else if (JOY_DOWN(adc))
        return JOYSTICK_DOWN;
    else {
         if (JOY_PUSHED())
            return JOYSTICK_PUSHED;
        else
            return JOYSTICK_CENTER;
    }
}// ebGetJoystickPosition




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: GetJoystickPosition.c,v $
 * Revision 1.2  2005/02/07 14:43:34  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
