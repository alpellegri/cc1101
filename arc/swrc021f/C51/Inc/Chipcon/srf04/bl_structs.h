/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***                                                                                *
 *      ***   +                                  CHIPCON SMARTRF04EB FIRMWARE                          *
 *      ***   + +   ***                           Bootloader Data Structures                           *
 *      ***   +++   ***                                                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Copyright Chipcon AS, 2005                                                                          *
 *******************************************************************************************************
 * This header file defines data structures that are shared between the application and the            *
 * bootloader. It also defines the address and size constants required for the bootloader and the      *
 * application to work together.                                                                       *
 *******************************************************************************************************
 * Compiler: Keil C51                                                                                  *
 * Target platform: SmartRF04EB (Silabs C8051F320)                                                     *
 *******************************************************************************************************
 * Revision history is located at the bottom of the file                                               *
 *******************************************************************************************************/
#ifndef BLSTRUCTS_H
#define BLSTRUCTS_H


#ifndef ASM_FILE
//include "common.h"
#include <Chipcon\srf04\common.h>


//-------------------------------------------------------------------------------------------------------
// USB device status information
typedef struct {
    BYTE usbState;
    BYTE pEpStatus[7]; // 0 = EP0, 1-3 = EP1-EP3 OUT, 4-6 = EP1-EP3 IN
    BYTE usbCommonInt;
    BYTE usbOutInt; 
    BYTE usbInInt;
} USB_INFO;
extern USB_INFO data usbInfo;

// Device state (used with USB_INFO.usbState)
#define DEV_ATTACHED            0x00
#define DEV_POWERED             0x01
#define DEV_DEFAULT             0x02
#define DEV_ADDRESS             0x03
#define DEV_CONFIGURED          0x04
#define DEV_SUSPENDED           0x05

// Endpoint status (used with USB_INFO.pEpStatus[n])
#define EP_IDLE                 0x00     // The endpoint is idle, or a setup token has been received
#define EP_TX                   0x01     // Setup IN data is transmitted automatically by the ISR
#define EP_RX                   0x02     // Setup OUT data is received automatically by the ISR
#define EP_HALT                 0x03     // The endpoint is halted (returns stalls to the host)
#define EP_STALL                0x04     // Send procedural stall in the next status phase
#define EP_MANUAL_RX            0x05     // Used to handle EP0 OUT data reception manually
#define EP_MANUAL_TX            0x06     // Used to handle EP0 IN data transmission manually
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// Information stored about the application
typedef struct {
    WORD chipType;
    BOOL appDescValid;
    BOOL appStarted;
} BOOTLOADER_INFO;
extern BOOTLOADER_INFO data blInfo;
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// USB setup header (used for all transactions on EP0)
// Make sure to update debug_handler.a51 if any changes are made to this structure
typedef struct {
    BYTE requestType;
    BYTE request;
    WORD value;
    WORD index;
    WORD length;
} SETUP_HEADER;
extern SETUP_HEADER data setupHeader;

// USB setup transfer buffer
typedef struct {
    BYTE *pBuffer;
    UINT16 bytesLeft;
} SETUP_DATA;
extern SETUP_DATA data setupData;
//-------------------------------------------------------------------------------------------------------


#endif // ASM_FILE


//-------------------------------------------------------------------------------------------------------
// Address and size constants for the memory shared between bootloader and applications

// Data variables (IDATA)
#define BOOTLOADER_TOTAL_MEM_SIZE   43
#define SETUP_HEADER_ADDR           0x08
#define SETUP_DATA_ADDR             (SETUP_HEADER_ADDR + sizeof(SETUP_HEADER))
#define USB_INFO_ADDR               (SETUP_DATA_ADDR + sizeof(SETUP_DATA))
#define BOOTLOADER_INFO_ADDR        (USB_INFO_ADDR + sizeof(USB_INFO))
#define SMART_VAR_ADDR              (BOOTLOADER_INFO_ADDR + sizeof(BOOTLOADER_INFO))
#define SMART_VAR_SIZE              (BOOTLOADER_TOTAL_MEM_SIZE - SMART_VAR_ADDR)

// Program and descriptors (CODE)
#define FLASH_PAGE_SIZE 512
#define APP_RESET_ADDR              0x0800
#define APP_DESC_LOCATOR_STARTADDR  (APP_RESET_ADDR + 0x0043 + 3)
#define APP_DESC_LOCATOR_STOPADDR   (APP_DESC_LOCATOR_STARTADDR + sizeof(APP_DESC_LOCATOR))
#define LAST_FLASH_PAGE_STARTADDR   0x3C00
#define LAST_FLASH_PAGE_STOPADDR    0x3DFF
#define FLASH_LOCK_BYTE_ADDR        LAST_FLASH_PAGE_STARTADDR
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
// The location of the USB PID in code memory
#define USB_PID_ADDR                (0x007E + 10)

// Is this an EB?
#define IS_DB_PID()                 (*((BYTE code*) (USB_PID_ADDR + 1)) == 0xDB)
#define IS_EB_PID()                 (*((BYTE code*) (USB_PID_ADDR + 1)) == 0xEB)
#define IS_DD_PID()                 (*((BYTE code*) (USB_PID_ADDR + 1)) == 0xDD)

// USB product IDs returned by the bootloader (little-endian)
#define CC2430DB_PID_LE             0x20DB
#define SRF04EB_PID_LE              0x20EB
#define SRF04DD_PID_LE              0x20DD
//-------------------------------------------------------------------------------------------------------


#endif


/*******************************************************************************************************
 * Revision history:
 *w
 * $Log: bl_structs.h,v $
 * Revision 1.1  2006/08/17 13:21:55  a0190596
 * Initial version in CVS
 *
 *
 *
 *******************************************************************************************************/

