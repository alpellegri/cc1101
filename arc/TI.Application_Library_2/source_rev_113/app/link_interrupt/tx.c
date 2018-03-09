/***********************************************************************************
    Filename: tx.c

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_mcu.h>
#include <hal_int.h>
#include <hal_board.h>
#include <hal_rf.h>
#include <cc2500.h>


//----------------------------------------------------------------------------------
//  Constants used in this file
//----------------------------------------------------------------------------------

// In this example, the threshold value is set such that an interrupt is generated
// when the FIFO is half full. It would be possible to set the threshold value
// differently - and thus get an interrupt earlier (meaning there are room for a
// few bytes in the FIFO, if interrupt on falling edge) or later (meaning that
// the FIFO is almost empty, if interrupt on falling edge).

#define FIFO_THRESHOLD       0x07
#define FIFO_THRESHOLD_BYTES 32
#define FIFO_SIZE            64


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
static volatile uint8  txStrobeNeeded   = TRUE;
static volatile uint8  nPacketsPending  = 0;
static volatile uint16 nBytesInFifo     = 0;


//----------------------------------------------------------------------------------
//  void txFifoHalfFull(void)
//
//  DESCRIPTION:
//    This function is called when the FIFO Threshold signal is deasserted, indicating
//    that the FIFO (in this example) is half full. Update the number of bytes
//    that are in the FIFO.
//----------------------------------------------------------------------------------
static void txFifoHalfFull(void)
{
    // This is potentially dangerous, as we cannot guarantee that
    // the ISR is invoked immediately after the condition occurs
    // or if someone is trying to write to the FIFO at the same time
    // (possible to get this interrupt if the radio is sending at the
    // same time). Should be handled with care in software.
    nBytesInFifo = FIFO_THRESHOLD_BYTES;
}


//----------------------------------------------------------------------------------
//  void txPacketSent(void)
//
//  DESCRIPTION:
//    This function is called every time the radio indicates that a packet has
//    been transmitted. Get the number of bytes in the FIFO and strobe TX if
//    there are packets to be sent.
//----------------------------------------------------------------------------------
static void txPacketSent(void)
{
    nPacketsPending--;

    // Get number of bytes in TXFIFO.
    // Note the CC1100/CC2500 errata concerning reading from the TXBYTES
    // register. However, since a packet has been transmitted, the radio
    // is no longer in TX state, since (in this particular application),
    // MCSM1.TXOFF_MODE is set to IDLE.
    nBytesInFifo = halRfReadStatusReg(CC2500_TXBYTES);

    if (nBytesInFifo & 0x80)
        // Oops! TX underflow. There is something seriously wrong
        // Don't try to do anything more.
        while(TRUE);

    if (nPacketsPending > 0 && nBytesInFifo > 0)
    {
        txStrobeNeeded = FALSE;
        halRfStrobe(CC2500_STX);
    }
    else
    {
        txStrobeNeeded = TRUE;
    }
}


//----------------------------------------------------------------------------------
//  void txInit(void)
//
//  DESCRIPTION:
//    Set up chip to operate in TX mode
//----------------------------------------------------------------------------------
void txInit(void)
{
    // Set TX FIFO threshold
    halRfWriteReg(CC2500_FIFOTHR, FIFO_THRESHOLD);

    // Set GDO0 to be TX FIFO threshold signal
    halRfWriteReg(CC2500_IOCFG0, 0x02);

    // Set up interrupt on GDO0
    halDigioIntSetEdge(&pinGDO0, HAL_DIGIO_INT_FALLING_EDGE);
    halDigioIntConnect(&pinGDO0, &txFifoHalfFull);
    halDigioIntEnable(&pinGDO0);

    // Set GDO2 to be packet sent signal
    halRfWriteReg(CC2500_IOCFG2, 0x06);

    // Set up interrupt on GDO2
    halDigioIntSetEdge(&pinGDO2, HAL_DIGIO_INT_FALLING_EDGE);
    halDigioIntConnect(&pinGDO2, &txPacketSent);
    halDigioIntEnable(&pinGDO2);
}


//----------------------------------------------------------------------------------
//  uint8 txSendPacket(uint8* data, uint8 length)
//
//  DESCRIPTION:
//    Send a packet over the air. Use flow control features of the CC1100/CC2500
//    to regulate the number of bytes that can be wirtten to the FIFO at ony time.
//    Return once the packet has been written to the FIFO (i.e. don't wait for the
//    packet to actually be sent).
//
//  ARGUMENTS:
//    data   - Data to send. First byte contains length byte
//    length - Total length of packet to send
//
//  RETURNS:
//    This function always returns 0.
//
//----------------------------------------------------------------------------------
uint8 txSendPacket(uint8* data, uint8 length)
{
    uint16 bytesRemaining = (uint16)length;
    uint16 bytesWritten   = 0;
    uint16 bytesToWrite;
    uint16 key;

    HAL_INT_LOCK(key);
    nPacketsPending++;
    HAL_INT_UNLOCK(key);

    while (bytesRemaining > 0)
    {
        HAL_INT_LOCK(key);

        bytesToWrite = MIN(FIFO_SIZE - nBytesInFifo, bytesRemaining);

        if (bytesToWrite == 0)
        {
            // Wait for available space in FIFO
            halMcuSetLowPowerMode(HAL_MCU_LPM_3);

            HAL_INT_UNLOCK(key);
        }
        else
        {
            // Write data fragment to FIFO
            halRfWriteFifo(&data[bytesWritten], bytesToWrite);
            nBytesInFifo += bytesToWrite;

            // We don't want to handle a pending FIFO half full interrupt now, since
            // the ISR will say that the FIFO is half full. However, as we have just
            // written to the FIFO, it might actually contain more bytes.
            halDigioIntClear(&pinGDO0);

            HAL_INT_UNLOCK(key);

            bytesWritten   += bytesToWrite;
            bytesRemaining -= bytesToWrite;
            if (txStrobeNeeded)
            {
                txStrobeNeeded = FALSE;
                halRfStrobe(CC2500_STX);
            }
        }
    }
    return(0);
}


