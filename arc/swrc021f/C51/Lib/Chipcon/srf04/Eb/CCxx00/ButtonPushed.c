/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements functionality getting the status of the push          *
 *      ***   +             button S1                                                                  *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     ButtonPushed.c                                                             *
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
UINT8 buttonActive;
BOOL buttonDebounced = TRUE;

#define   BUTTON_TIMEOUT   100
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  BOOL ebButtonPushed(void)
//
//  DESCRIPTION:
//      This function detects if the button is being pushed. The function implements software de-bounce.
//    
//  RETURN TYPE:
//      BOOL
//          TRUE: Button is being pushed
//          FALSE: Button is not being pushed
//-------------------------------------------------------------------------------------------------------
BOOL ebButtonPushed(void) {
    if(BUTTON_PUSHED()) {
        if(buttonDebounced) {
            if(buttonActive++ == BUTTON_TIMEOUT) {
                buttonDebounced = FALSE;
                buttonActive = 0;
                return TRUE;
            }
        }
    } else {
        if(buttonActive++ == BUTTON_TIMEOUT) {
            buttonDebounced = TRUE;
            buttonActive = 0;
        }
    }
    return FALSE;
}// ebButtonPushed




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: ButtonPushed.c,v $
 * Revision 1.3  2005/07/11 12:29:16  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:43:27  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
