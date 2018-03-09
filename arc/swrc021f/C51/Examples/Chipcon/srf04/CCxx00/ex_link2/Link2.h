/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains all defines and function prototypes necessary to        *
 *      ***   +             run the Link2 example.                                                     *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Link2.h                                                                    *
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
#define LINE_SIZE   16
#define ADDR        0

// Defines used by the displayMenu() function to keep track of the current menu entry
#define PACKET_LENGTH       0
#define NUMBER_OF_PACKETS   1
#define RADIO_MODE          2
#define START               3

// defines used for assigning values to the variables in the MENU_DATA struct.
#define TX  0
#define RX  1

// defines used in the state machine in the main loop
#define TX_START    0
#define TX_WAIT     1
#define RX_START    2
#define RX_WAIT     3
#define SETUP       4

#define FIFO_SIZE   64

#define SYNC            0
#define END_OF_PACKET   1 

#define CRC_OK  0x80 

#define AVAILABLE_BYTES_IN_TX_FIFO  60
#define BYTES_IN_RX_FIFO            60

#define PACKET_INT      P0_6   

// Struct. used to hold information of each and every menu entry
typedef struct MENU_ITEM {
    UINT8 menuName[LINE_SIZE + 1];  // Menu Name
    UINT16 *pValue;                 // Current value
    UINT8 min;                      // Min value possible
    UINT16 max;                     // Max value possible
} MENU_ITEM;

// Each of the meny entries will have a pointer to one of these variables
typedef struct MENU_DATA{
    UINT16 packetLength;
    UINT16 numberOfPackets;
    UINT16 radioMode;
} MENU_DATA;          


// Struct. used to hold information used for packet handling in TX
typedef struct TX_DATA {
    UINT16 bytesLeft;           // Used to keep track of how many bytes are left to be written to 
                                // the TX FIFO
    UINT8 iterations;           // For packets greater than 64 bytes, this variable is used to keep 
                                // track of how many time the TX FIFO should be re-filled to its limit 
    BOOL writeRemainingDataFlag;// When this flag is set, the TX FIFO should not be filled entirely
    BOOL packetSentFlag;        // Flag set when GDO0 indicates that the packet is sent
    UINT8 *pBufferIndex;        // Pointer to current position in the txBuffer 
    UINT16 packetsSent;         // Number of packets transmitted
} TX_DATA; 

// Struct. used to hold information used for packet handling in RX
typedef struct RX_DATA {
    UINT16 bytesLeft;           // Used to keep track of how many bytes are left to be read from the RX FIFO
    BOOL packetReceivedFlag;    // Flag set when GDO0 indicates that a packet is received
    BOOL syncOrEndOfPacket;     // Flag used to determine if the interrupt is caused by a rising or
                                // a falling edge
    UINT8 *pBufferIndex;        // Pointer to current position in the rxBuffer 
    UINT8 lengthByte;           // LengthByte (This example require variable packet length mode)
    BOOL crcOK;                 // CRC_OK flag from status byte in RX FIFO
    UINT16 packetsReceived;     // Number of packets received
} RX_DATA;

//-------------------------------------------------------------------------------------------------------
//  Global Variables

extern RF_SETTINGS code rfSettings;

extern BYTE code paTable;

extern MENU_DATA xdata menuData;

extern MENU_ITEM code menuTable[];

//Variable to keep track of the current menu entry (can have a value between 0 and 5)
extern UINT8 index;  

// Byte array used by the intToAscii(UINT16 value) function
extern UINT8 xdata asciiString[6];
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Function declarations

// Tx related functions
void createDataPacket(void);

// Menu related functions
void parseMenu(UINT8 joystickPosition);
void handleAndDisplayMenu(void);

// Function called in the main loop
void intToAscii(UINT16 value);
//-------------------------------------------------------------------------------------------------------




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: Link2.h,v $
 * Revision 1.3  2006/04/11 13:07:05  a0190596
 * Removed iterations from the RX_DATA struct.
 * Added and changed some defines
 *
 * Revision 1.2  2006/03/31 13:19:47  a0190596
 * AVAILABLE_BYTES_IN_TX_FIFO  changed from 60 to 59
 *
 * Revision 1.1  2006/03/17 10:40:24  a0190596
 * Initial version in CVS
 *
 * 
 ******************************************************************************************************/

