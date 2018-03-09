/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements functionality getting the status of the push          *
 *      ***   +             button S1                                                                  *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     JoyPushed.c                                                                *
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




//-------------------------------------------------------------------------------------------------------
//  Variables and defines
UINT8 joyActive;
BOOL joyDebounced = TRUE;

#define   JOYSTICK_TIMEOUT   100
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  BOOL ebJoyPushed(void)
//
//  DESCRIPTION:
//      This function detects if the joystick button is being pushed. 
//      The function implements software de-bounce.
//     
//  RETURN TYPE:
//      BOOL
//          TRUE: Joystick button is being pushed
//          FALSE: Joystick button is not being pushed
//-------------------------------------------------------------------------------------------------------
BOOL ebJoyPushed(void) {
    if(JOY_PUSHED()) {
        if(joyDebounced) {
            if(joyActive++ == JOYSTICK_TIMEOUT) {
                joyDebounced = FALSE;
                joyActive = 0;
                return TRUE;
            }
        }
    } else {
        if(joyActive++ == JOYSTICK_TIMEOUT) {
            joyDebounced = TRUE;
            joyActive = 0;
        }
    }
    return FALSE;
}// ebJoyPushed




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: JoyPushed.c,v $
 * Revision 1.3  2005/07/11 12:29:23  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:43:41  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/