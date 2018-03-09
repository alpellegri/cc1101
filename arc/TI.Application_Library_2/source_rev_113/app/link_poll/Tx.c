/***********************************************************************************
    Filename: tx.c

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_board.h>
#include <hal_timer.h>
#include <hal_int.h>
#include <hal_led.h>
#include <hal_mcu.h>
#include <hal_rf.h>
#include <cc2500.h>


typedef struct
{
    uint8 txInProgress;        // Indicates that a packet is being written to the FIFO
    uint8 txBytesWritten;      // Variable to keep track of the data in txBuffer
    uint8 txBytesRemaining;    // Variable holding information on number og bytes left in txBuffer
    uint8 *pTxBuffer;          // Pointer to txBuffer
    uint8 pktsPending;         // Number of packets pending to be written
} PKT_DATA;


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
static PKT_DATA pktData;


//----------------------------------------------------------------------------------
//  void txPktHandler(void)
//
//  DESCRIPTION:
//    This function is called every time a timer interrupt occurs (every 200 us).
//    Continue only if a packet is in progress or if packets are pending to be
//    sent (i.e. written to the FIFO, but not yet sent). If so, read the chip status
//    byte. Every time the status byte indicates that there is free space in the
//    TX FIFO, bytes from txBuffer and written to the TX FIFO until the whole
//    packet is written (pktData.txBytesRemaining = 0).
//
//    When the status byte indicates that the radio is in IDLE state and if there
//    are packets to send, strobe TX.
//
//  NOTE:
//    Note that the status byte polling cannot fully be trusted (see the
//    Errata Note). Thus read the status byte until the same value is read twice.
//----------------------------------------------------------------------------------
static void txPktHandler(void)
 {
    uint8 status;
    uint8 status2;
    uint8 freeSpaceInFifo;

    // Don't do anything unless a packet is to be transmitted
    if (!pktData.txInProgress && !pktData.pktsPending)
    {
        return;
    }

    // Get state of chip
    status  = halRfGetTxStatus();
    status2 = halRfGetTxStatus();
    while (status != status2)
    {
        status  = status2;
        status2 = halRfGetTxStatus();
    }

    switch (status & CC2500_STATUS_STATE_BM)
    {
        case CC2500_STATE_IDLE:

           // Strobe TX if there are packets pending in the FIFO
           if (pktData.pktsPending > 0)
           {
               pktData.pktsPending--;
               halRfStrobe(CC2500_STX);
           }
           // fallthrough (no break)

        case CC2500_STATE_CALIBRATE:
        case CC2500_STATE_TX:

            // If there's anything to transfer..
            if (freeSpaceInFifo = MIN(pktData.txBytesRemaining, status & CC2500_STATUS_FIFO_BYTES_AVAILABLE_BM))
            {
                halRfWriteFifo(&pktData.pTxBuffer[pktData.txBytesWritten], freeSpaceInFifo);
                pktData.txBytesWritten   += freeSpaceInFifo;
                pktData.txBytesRemaining -= freeSpaceInFifo;

                // Notify the application if all bytes in the packet has been written to the TX FIFO
                if (pktData.txBytesRemaining == 0)
                {
                    pktData.txInProgress = FALSE;
                }
            }
            break;

        case CC2500_STATE_TX_UNDERFLOW:

            // Flush and clean up
            pktData.pktsPending = 0;
            pktData.txInProgress = FALSE;
            halRfStrobe(CC2500_SFTX);
            break;

        default:
            break;
    }
}// txPktHandler


//-------------------------------------------------------------------------------------------------------
//  void pktDataInit(void)
//
//  DESCRIPTION:
//    Function to initialize the pktData structure.
//-------------------------------------------------------------------------------------------------------
static void pktDataInit(void)
{
    pktData.pTxBuffer        = NULL;
    pktData.txInProgress     = FALSE;
    pktData.txBytesRemaining = 0;
    pktData.txBytesWritten   = 0;
    pktData.pktsPending      = 0;
}


//-------------------------------------------------------------------------------------------------------
//  void txInit(void)
//
//  DESCRIPTION:
//    Set up chip to operate in TX mode
//-------------------------------------------------------------------------------------------------------
void txInit(void)
{
    // Initialize packet data
    pktDataInit();

    // Configure interrupts
    halTimerInit(200);
    halTimerIntConnect(&txPktHandler);
    halTimerIntEnable();
}


//----------------------------------------------------------------------------------
//  void txSendPacket(uint8* data, uint8 length)
//
//  DESCRIPTION:
//    This function initiates a new packet transfer. Set up the packet structure
//    and wait for the packet to be sent.
//
//  ARGUMENTS:
//    data   - Data to send. First byte contains length byte
//    length - Total length of packet to send
//
//  RETURNS:
//    This function always returns 0.
//----------------------------------------------------------------------------------
uint8 txSendPacket(uint8* data, uint8 length)
{
    uint16 key = halIntLock();

    pktData.pTxBuffer        = data;
    pktData.txBytesRemaining = length;
    pktData.txBytesWritten   = 0;
    pktData.txInProgress     = TRUE;
    pktData.pktsPending++;

    halIntUnlock(key);

    while(pktData.txInProgress)
        halMcuSetLowPowerMode(HAL_MCU_LPM_1);

    return(0);
}

