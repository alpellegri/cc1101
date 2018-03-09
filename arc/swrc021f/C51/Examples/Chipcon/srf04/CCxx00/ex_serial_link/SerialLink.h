/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains all defines and function prototypes necessary to        *
 *      ***   +             run the SerialLink example.                                                *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SerialLink.h                                                               *
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
 *******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>




//-------------------------------------------------------------------------------------------------------
// Defines

#define MODE_NOT_SET        0
#define TX                  1
#define RX                  2  

#define SYNC_LENGTH         4
#define PREAMBLE_LENGTH     4

#define START_OF_PAYLOAD    (SYNC_LENGTH + PREAMBLE_LENGTH)

#define CRC_ENABLE          TRUE
#define CRC_DISABLE         FALSE

#define CRC1                0x00    // Dummy bytes to make room for CRC bytes in the packet
#define CRC0                0x00

#define VARIABLE_LENGTH     0       // First byte after sync word is interpreted as the length byte

#define SYNC3               0xD3
#define SYNC2               0x91
#define SYNC1               0xD3
#define SYNC0               0x91
//-------------------------------------------------------------------------------------------------------


//-------------------------------------------------------------------------------------------------------
//  Global Variables

extern RF_SETTINGS code rfSettings;

extern BYTE code paTable;
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Function declarations

void parseMenu(UINT8 joystickPosition);
void intToAscii(UINT32 value);
//-------------------------------------------------------------------------------------------------------




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: SerialLink.h,v $
 * Revision 1.1  2006/10/06 13:28:11  a0190596
 * Initial version in CVS.
 *
 * 
 ******************************************************************************************************/