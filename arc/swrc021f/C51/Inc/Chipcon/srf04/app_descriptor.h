/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***                                                                                *
 *      ***   +                                  CHIPCON SMARTRF04EB FIRMWARE                          *
 *      ***   + +   ***                             Application Descriptor                             *
 *      ***   +++   ***                                                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:     See end of file                                                               *
 *******************************************************************************************************
 * The application descriptor tells the bootloader where to find the actual USB descriptors to be      *
 * reported to the PC. It also provides hooks for the bootloader to forward interrupts and USB         *
 * requests to the application, and perform device detection during startup. A "magic word" must is    *
 * placed at address C:0x0846 for the descriptor to be accepted. The "magic word" is followed by a     *
 * pointer to the APP_DESC pointer This allows the bootloader to start the application immediately.    *
 ******************************************************************************************************/

#ifndef APPDESC_H
#define APPDESC_H


#ifndef ASM_FILE
#include "usb_descriptor.h"


//-------------------------------------------------------------------------------------------------------
// The number of BULK endpoints (use a modified copy of this file when ENDPOINT_COUNT != 2)
#define ENDPOINT_COUNT 2

// The application descriptor
typedef struct {
    WORD firmwareId;
    WORD firmwareRev;
    void code *pChipDetectFunc;
    void code *pHandleCommonInt;
    void code *pHandleVendorOrClassRequest;
    void code *pHandleInEndpoints;
    void code *pHandleOutEndpoints;
    USB_CONFIGURATION_DESCRIPTOR configDesc;
    USB_INTERFACE_DESCRIPTOR interfaceDesc;
    USB_ENDPOINT_DESCRIPTOR pEndpointDescs[ENDPOINT_COUNT];
} APP_DESC;

// The application descriptor location struct
typedef struct {
    BYTE pMagicWord[3];
    APP_DESC code *pAppDesc;
} APP_DESC_LOCATOR;
extern APP_DESC_LOCATOR code appDescLocator;
//-------------------------------------------------------------------------------------------------------


#endif // ASM_FILE


//-------------------------------------------------------------------------------------------------------
// The "magic word" to be used with APP_DESC_LOCATOR.pMagicWord (do not change!!!)
#define MAGIC_WORD_0    0xE1
#define MAGIC_WORD_1    0x8C
#define MAGIC_WORD_2    0x64
//-------------------------------------------------------------------------------------------------------


#endif


/*******************************************************************************************************
 * Revision history:
 *
 * $Log: app_descriptor.h,v $
 * Revision 1.1  2006/08/17 13:21:54  a0190596
 * Initial version in CVS
 *
 *
 *
 *******************************************************************************************************/
