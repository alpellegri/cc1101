/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file should be included when creating bootloader compliant            *
 *      ***   +             applications.                                                              *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     bootloader_reservations.c                                                  *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  JOL                                                                        *
 *******************************************************************************************************
 * Revision history:        See end of file                                                            *
 ******************************************************************************************************/
#include <Chipcon\srf04\bl_structs.h>




//-------------------------------------------------------------------------------------------------------
// Memory shared between bootloader and applications

// Buffer for current device request information
SETUP_HEADER data setupHeader _at_ SETUP_HEADER_ADDR;
SETUP_DATA data setupData _at_ SETUP_DATA_ADDR;

// USB status information (read only)
USB_INFO data usbInfo _at_ USB_INFO_ADDR;

// Bootloader status information (read only)
BOOTLOADER_INFO data blInfo _at_ BOOTLOADER_INFO_ADDR;

// Additional RAM used by the bootloader
BYTE data pBlSmartVar[SMART_VAR_SIZE] _at_ SMART_VAR_ADDR;

// Bootloader code at the start of the flash
BYTE code pBootloader[APP_RESET_ADDR - 0] _at_ 0x0000;
//-------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: bootloader_reservations.c,v $
 * Revision 1.3  2006/08/17 13:29:36  a0190596
 * Changed include file
 *
 * Revision 1.2  2005/02/07 14:43:05  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
