/***********************************************************************************
    Filename: rx.c

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_board.h>
#include <hal_timer.h>
#include <hal_mcu.h>
#include <hal_rf.h>
#include <cc2500.h>


typedef struct
{
    uint16 rxTimeout;          // Variable initialized by the timeout variable in pktStartRx()
                               // Decremented every 200 us. RX mode is terminated when it reaches 0
    uint8  rxTimeoutActive;    // Set in the pktStartRx function if timeout != 0. Cleard in
                               // pktRxHandler() when pktData.rxTimeout reaches 0.
    uint8  rxInProgress;       // Indicates that a packet is being received
    uint16 rxBytesRead;        // Variable to keep track of the data in rxBuffer
    uint16 rxBytesRemaining;   // Variable to keep track of how many bytes are left to be received
    uint8  *pRxBuffer;         // Pointer to rxBuffer
    uint8  length;             // Length of packet
    uint8  lengthByteRead;     // Flag set to 1 when the length byte has been read
    uint8  status;             // Return value/status from reception
} PKT_DATA;


//----------------------------------------------------------------------------------
//  Constants used in this file
//----------------------------------------------------------------------------------
#define RX_OK                0
#define RX_LENGTH_VIOLATION  1
#define RX_CRC_MISMATCH      2
#define RX_FIFO_OVERFLOW     3
#define RX_TIMEOUT           4


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
static PKT_DATA pktData;


//----------------------------------------------------------------------------------
//  void rxPktHandler(void)
//
//  DESCRIPTION:
//    This function is called every time a timer interrupt occurs (every 200 us).
//    Continue only if a packet is in progress and if the receiver has not timed
//    out. In case of timeout, terminate reception and flush the RX FIFO. If not,
//    read the chip status byte. Every time the status byte indicates that there
//    are available bytes in the RX FIFO, bytes are read from the RX FIFO and
//    written to rxBuffer. This procedure is repeated until the whole packet is
//    received rxBytesRemaining = 0). If the status byte indicates that there has
//    been an RX FIFO overflow, reception is terminated.
//
//  NOTE:
//    Due to the RX FIFO problem, the RX FIFO is not emptied before the last
//    byte has been received (see Errata Note).
//
//    Also note that the status byte polling cannot fully be trusted (see the
//    Errata Note). Thus read the status byte until the same value is read twice.
//----------------------------------------------------------------------------------
static void rxPktHandler(void)
{
    uint8  status;
    uint8  status2;
    uint16 bytesInFifo;

    // Don't do anything unless a packet is to be received
    if (!pktData.rxInProgress)
    {
        return;
    }

    // Timeout handling
    if (pktData.rxTimeoutActive)
    {
        if (pktData.rxTimeout)
        {
            pktData.rxTimeout--;
        }
        else
        {
            pktData.rxInProgress = FALSE;
            pktData.status       = RX_TIMEOUT;
            halRfStrobe(CC2500_SIDLE);
            halRfStrobe(CC2500_SFRX);
            return;
        }
    }

    // Which state?
    status  = halRfGetRxStatus();
    status2 = halRfGetRxStatus();
    while (status != status2)
    {
        status  = status2;
        status2 = halRfGetRxStatus();
    }

    switch (status & CC2500_STATUS_STATE_BM)
    {
        case CC2500_STATE_IDLE:
        case CC2500_STATE_RX:

            // If there's anything in the RX FIFO....
            if (bytesInFifo = (status & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM))
            {
                // Start by getting the packet length
                if (pktData.lengthByteRead == FALSE)
                {
                    // Make sure that the RX FIFO is not emptied
                    // (see the CC1100 or 2500 Errata Note)
                    if (bytesInFifo > 1)
                    {
                        pktData.length           = halRfReadReg(CC2500_RXFIFO);
                        pktData.lengthByteRead   = TRUE;
                        pktData.rxBytesRemaining = pktData.length + 2; // Packet Length + 2 appended bytes
                        pktData.rxBytesRead      = 0;
                        bytesInFifo--;
                    }
                    else
                    {
                        // Need more data in FIFO before reading the length byte
                        // (the first byte of the packet)
                        break;
                    }
                }

                // Make sure that the RX FIFO is not emptied
                // (see the CC1100 or 2500 Errata Note)
                if ((bytesInFifo > 1) && (bytesInFifo != pktData.rxBytesRemaining))
                {
                    // Leave one byte in FIFO
                    bytesInFifo--;
                }
                else if ((bytesInFifo <= 1) && (bytesInFifo != pktData.rxBytesRemaining))
                {
                    // Need more data in FIFO before reading additional bytes
                    break;
                }

                // Read from RX FIFO and store the data in rxBuffer
                halRfReadFifo(&(pktData.pRxBuffer[pktData.rxBytesRead]), bytesInFifo);
                pktData.rxBytesRead      += bytesInFifo;
                pktData.rxBytesRemaining -= bytesInFifo;

                // Done?
                if ((pktData.rxBytesRemaining == 0) && (pktData.lengthByteRead))
                {
                    pktData.rxInProgress = FALSE;
                    pktData.status       = RX_OK;
                }
            }
            break;

        case CC2500_STATE_RX_OVERFLOW:

            pktData.rxInProgress = FALSE;
            pktData.status       = RX_FIFO_OVERFLOW;
            halRfStrobe(CC2500_SFRX);
            break;

        default:
            break;

    }
}// rxPktHandler


//----------------------------------------------------------------------------------
//  void pktDataInit(void)
//
//  DESCRIPTION:
//    Function to initialize the pktData structure.
//----------------------------------------------------------------------------------
static void pktDataInit(void)
{
    pktData.rxTimeout        = 0;
    pktData.rxTimeoutActive  = FALSE;
    pktData.rxBytesRead      = 0;
    pktData.rxBytesRemaining = 0;
    pktData.pRxBuffer        = NULL;
    pktData.rxInProgress     = FALSE;
    pktData.lengthByteRead   = FALSE;
    pktData.length           = 0;
}


//----------------------------------------------------------------------------------
//  void rxInit(void)
//
//  DESCRIPTION:
//    Set up chip to operate in RX mode
//----------------------------------------------------------------------------------
void rxInit(void)
{
    // Initialize packet data
    pktDataInit();

    // Configure interrupts
    halTimerInit(200);
    halTimerIntConnect(&rxPktHandler);
    halTimerIntEnable();
}

//----------------------------------------------------------------------------------
//  uint8 rxRecvPacket(uint8* data, uint8* length, uint16 timeout)
//
//  DESCRIPTION:
//    Receive packet from radio. It sets the CC1100/CC2500 in RX mode and waits for
//    the packet to arrive (handled by the timer ISR).
//
//  ARGUMENTS:
//    data    - Pointer to where to write the incoming packet
//    length  - Pointer to where the length of the packet should be written
//    timeout - Indicate for how many intervals of 200 microsecond to wait
//              before a packet has arrived. Note that if the reception times
//              out during handling of an incoming oacket, that packet is
//              discarded. If timeout is 0, wait forever.
//
//  RETURNS:
//    RX_OK (0)         if a packet was received successfully.
//    RX_FIFO_OVERFLOW  if chip is in overflow state.
//    RX_CRC_MISMATCH   if the CRC of the packet is not OK.
//    RX_TIMEOUT        if reception timed out.
//----------------------------------------------------------------------------------
uint8 rxRecvPacket(uint8* data, uint8* length, uint16 timeout)
{
    if (timeout)
    {
        pktData.rxTimeout       = timeout;
        pktData.rxTimeoutActive = TRUE;
    }
    else
    {
        pktData.rxTimeoutActive = FALSE;
    }

    pktData.pRxBuffer = data;
    pktData.lengthByteRead = FALSE;
    pktData.status = RX_OK;

    halRfStrobe(CC2500_SRX);
    pktData.rxInProgress = TRUE;

    // Wait for packet
    while (pktData.rxInProgress)
        halMcuSetLowPowerMode(HAL_MCU_LPM_1);

    // Check CRC
    if (pktData.status == RX_OK)
    {
        if (pktData.pRxBuffer[pktData.length + 1] & CC2500_LQI_CRC_OK_BM)
        {
            *length = pktData.length;
        }
        else
        {
            *length = 0;
            pktData.status = RX_CRC_MISMATCH;
        }
    }

    return(pktData.status);

}



