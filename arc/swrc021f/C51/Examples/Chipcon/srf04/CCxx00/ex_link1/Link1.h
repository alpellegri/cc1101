/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains all defines and function prototypes necessary to        *
 *      ***   +             run the Link1 example.                                                     *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Link1.h                                                                    *
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
#define WHITENING           2
#define RADIO_MODE          3
#define START               4

// defines used for assigning values to the variables in the MENU_DATA struct.
#define WHITENING_DISABLED  FALSE
#define WHITENING_ENABLED   TRUE
#define TX                  0
#define RX                  1

// defines used in the state machine in the main loop
#define TX_START            0
#define TX_WAIT             1
#define RX_START            2
#define RX_WAIT             3
#define SETUP               4

// Bit masks corresponding to STATE[2:0] in the status byte returned on MISO
#define CC2500_STATE_BM                 0x70
#define CC2500_FIFO_BYTES_AVAILABLE_BM  0x0F
#define CC2500_STATE_TX_BM              0x20
#define CC2500_STATE_TX_UNDERFLOW_BM    0x70
#define CC2500_STATE_RX_BM              0x10
#define CC2500_STATE_RX_OVERFLOW_BM     0x60
#define CC2500_STATE_IDLE_BM            0x00

// In the status byte returned on MISO there are 4 bits (FIFO_BYTES_AVAILABLE[3:0])
// giving the number of bytes available in the RX FIFO or free bytes in the TX FIFO
#define PKT_MAX_BYTES_PER_TRANSFER      16

// SPI related defines
#define CSn     NSSMD0
#define LOW     0
#define HIGH    1

#define CRC_OK  0x80  

// Macro which returns the lowest number (n or m)
#define MIN(n,m) (((n) < (m)) ? (n) : (m))

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
    UINT16 whitening;
    UINT16 radioMode;
} MENU_DATA;

// This struct contains all necessary variables for operating the pktRxHandler and pktTxHandler
typedef struct {
    BOOL txInProgress;              // Set right before a STX strobe is transmitted. Cleared when 
                                    // radio enters IDLE mode (pktTxHandler())
    UINT16 txStartupTimeout;        // Variable to keep track of bytes left in the txBuffer
    BOOL txStartupTimeoutActive;    // Set when pktStartTx() is being called. Cleared in the 
                                    // pktTxHandler() when pktData.txStartupTimeout == 0 
    UINT16 rxTimeout;               // Variable initialized by the timeout variable in pktStartRx()
                                    // Decremented every 200 us. RX mode is terminated when it reaches 0       
    BOOL rxTimeoutActive;           // Set in the pktStartRx function if timeout != 0. Cleard in 
                                    // pktRxHandler() when pktData.rxTimeout reaches 0.
    BOOL rxTimeoutReached;          // Set when the Rx timeout is reached and RX mode is terminated.
    UINT8 txPosition;               // Variable to keep track of the data in txBuffer
    UINT16 rxPosition;              // Variable to keep track of the data in rxBuffer
    UINT8 txBytesLeft;              // Variable holding information on number og bytes left in txBuffer
    UINT16 rxBytesLeft;             // Variable to keep track of how many bytes are left to be received
    BYTE xdata *pTxBuffer;          // Pointer to txBuffer
    BYTE xdata *pRxBuffer;          // Pointer to rxBuffer
    BOOL pktTransmitted;            // Flag set when there are no bytes left in the txBuffer
    BOOL pktReceived;               // Flag set when a packet is received
    BOOL lengthByteRead;            // Flag set to 1 when the length byte has been read
    BOOL txUnderflowFlag;           // Flag set to 1 when the TX FIFO is flushed due to underflow
} PKT_DATA;
          
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  Global Variables

extern RF_SETTINGS code rfSettings;

extern BYTE code paTable;

extern BYTE xdata txBuffer[258];   // 1 length byte + 255 bytes payload + 2 status bytes

extern MENU_DATA xdata menuData;

//Variable to keep track of the current menu entry (can have a value between 0 and 5)
extern UINT8 index;  

// Byte array used by the intToAscii(UINT16 value) function
extern UINT8 xdata asciiString[6];

extern PKT_DATA xdata pktData;
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Function declarations

// Tx related functions
void createDataPacket(void);
void pktStartTx(UINT16 timeout);
void pktTxHandler(void);
BYTE spiGetTxStatus(void);

// Rx related functions
void pktStartRx(UINT16 timeout);
void pktRxHandler(void);
BYTE spiGetRxStatus(void);

// Menu related functions
void parseMenu(UINT8 joystickPosition);
void handleAndDisplayMenu(void);

// Function called in the main loop
void intToAscii(UINT16 value);

// Function used to initial the pktData struct.
void pktDataInit(void);
//-------------------------------------------------------------------------------------------------------




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: Link1.h,v $
 * Revision 1.5  2007/01/25 12:14:35  a0190596
 * no message
 *
 * Revision 1.4  2006/11/14 13:37:16  a0190596
 * Removed FEC option
 *
 * Revision 1.3  2006/03/17 10:20:05  a0190596
 * New file structure might have caused variables, defines, and functions to move.
 *
 * Revision 1.2  2005/06/14 11:08:07  sna
 * no message
 *
 * Revision 1.1  2005/04/20 12:41:42  sna
 * Initial version in CVS
 *
 *
 *
 ******************************************************************************************************/

