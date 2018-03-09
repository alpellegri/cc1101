/***********************************************************************************
    Filename: rx.c

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
#define RX_OK                0
#define RX_LENGTH_VIOLATION  1
#define RX_CRC_MISMATCH      2
#define RX_FIFO_OVERFLOW     3


// In this example, the threshold value is set such that an interrupt is generated
// when the FIFO is half full. It would be possible to set the threshold value
// differently - and thus get an interrupt earlier (meaning there are a few bytes
// available in the FIFO, if interrupt on rising edge) or later (meaning that
// the FIFO is almost full, if interrupt on rising edge).

#define FIFO_THRESHOLD        0x07
#define FIFO_THRESHOLD_BYTES  32
#define FIFO_SIZE             64


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
static volatile uint8 dataPending;
static volatile uint8 packetComplete;


//----------------------------------------------------------------------------------
//  void rxFifoHalfFull(void)
//
//  DESCRIPTION:
//    This function is called when the FIFO Threshold signal is asserted, indicating
//    that the FIFO (in this example) is half full. Set a flag indicating that there
//    is data in the FIFO.
//----------------------------------------------------------------------------------
void rxFifoHalfFull(void)
{
    dataPending = TRUE;
}


//----------------------------------------------------------------------------------
//  void rxPacketRecvd(void)
//
//  DESCRIPTION:
//    This function is called when a complete packet has been received. Set a flag
//    indicating that radio has received a complete packet.
//----------------------------------------------------------------------------------
void rxPacketRecvd(void)
{
    packetComplete = TRUE;
}


//----------------------------------------------------------------------------------
//  void rxInit(void)
//
//  DESCRIPTION:
//    Set up chip to operate in RX mode
//----------------------------------------------------------------------------------
void rxInit(void)
{
    // Set RX FIFO threshold
    halRfWriteReg(CC2500_FIFOTHR, FIFO_THRESHOLD);

    // Set GDO0 to be RX FIFO threshold signal
    halRfWriteReg(CC2500_IOCFG0, 0x00);

    // Set up interrupt on GDO0
    halDigioIntSetEdge(&pinGDO0, HAL_DIGIO_INT_RISING_EDGE);
    halDigioIntConnect(&pinGDO0, &rxFifoHalfFull);
    halDigioIntEnable(&pinGDO0);

    // Set GDO2 to be packet received signal
    halRfWriteReg(CC2500_IOCFG2, 0x06);

    // Set up interrupt on GDO2
    halDigioIntSetEdge(&pinGDO2, HAL_DIGIO_INT_FALLING_EDGE);
    halDigioIntConnect(&pinGDO2, &rxPacketRecvd);
    halDigioIntEnable(&pinGDO2);
}


//----------------------------------------------------------------------------------
//  uint8 rxRecvPacket(uint8* data, uint16* length)
//
//  DESCRIPTION:
//    Receive packet from radio. Waits for either a FIFO half full event or
//    a packet received event and reads data from the RX FIFO accordingly.
//    Returns when a complete packet is received.
//
//  ARGUMENTS:
//    data    - Pointer to where to write the incoming packet payload
//    length  - Pointer to where to write the length of the packet payload
//
//  RETURNS:
//    RX_OK                - packet was received successfully
//    RX_LENGTH_VIOLATION  - length of the packet is illegal
//    RX_CRC_MISMATCH      - claculated CRC does not match packet CRC
//
//----------------------------------------------------------------------------------
uint8 rxRecvPacket(uint8* data, uint8* length)
{
    uint8 done = FALSE;
    uint8 lengthByteRead = FALSE;

    uint8 appendStatus[2];
    uint8 bytesRead = 0;
    uint8 bytesRemaining = 0;
    uint16 key;

    // Reset state and set radio in RX mode
    // Safe to set states, as radio is IDLE
    halDigioIntClear(&pinGDO2);
    halDigioIntClear(&pinGDO0);
    packetComplete = FALSE;
    dataPending    = FALSE;
    halRfStrobe(CC2500_SRX);

    while (!done)
    {
        // Wait for further action
        // Be careful here. This is actually a critical section. If you get
        // the interrupt AFTER checking the two booleans, but BEFORE setting
        // the low power mode, you might get stuck!
        HAL_INT_LOCK(key);
        if (!packetComplete && !dataPending)
        {
            halMcuSetLowPowerMode(HAL_MCU_LPM_3);  // Will turn global interrupts on
        }

        // An interrupt has occured. Take the appropriate action
        // according to the state of the system.
        HAL_INT_LOCK(key);
        if (packetComplete && !lengthByteRead)
        {
            packetComplete = FALSE;

            halRfReadFifo(length, 1);
            lengthByteRead = TRUE;

            if (*length == 0 || *length > FIFO_SIZE)
            {
                halRfStrobe(CC2500_SIDLE);
                halRfStrobe(CC2500_SFRX);
                HAL_INT_UNLOCK(key);
                return(RX_LENGTH_VIOLATION);
            }

            // Get the complete packet from the FIFO
            halRfReadFifo(data, *length);
            done = TRUE;
        }
        else if (packetComplete && lengthByteRead)
        {
            packetComplete = FALSE;

            halRfReadFifo(&data[bytesRead], bytesRemaining);
            done = TRUE;
        }
        else if (dataPending && !lengthByteRead)
        {
            dataPending = FALSE;

            halRfReadFifo(length, 1);
            lengthByteRead = TRUE;

            if (*length == 0)
            {
                halRfStrobe(CC2500_SIDLE);
                halRfStrobe(CC2500_SFRX);
                HAL_INT_UNLOCK(key);
                return(RX_LENGTH_VIOLATION);
            }

            // Read remaining bytes in FIFO, but don't empty the
            // FIFO because of RX FIFO behaviour
            halRfReadFifo(data, FIFO_THRESHOLD_BYTES - 2);
            bytesRead += FIFO_THRESHOLD_BYTES - 2;
            bytesRemaining = *length - bytesRead;
        }
        else if (dataPending && lengthByteRead)
        {
            dataPending = FALSE;

            // Read remaining bytes in FIFO, but don't empty the
            // FIFO because of RX FIFO behaviour
            halRfReadFifo(&data[bytesRead], FIFO_THRESHOLD_BYTES - 1);
            bytesRead += FIFO_THRESHOLD_BYTES - 1;
            bytesRemaining = *length - bytesRead;
        }

        halDigioIntClear(&pinGDO0);
        HAL_INT_UNLOCK(key);
    }

    // Get the appended status bytes [RSSI, LQI]
    halRfReadFifo(appendStatus, 2);

    // Check CRC
    if ((appendStatus[1] & CC2500_LQI_CRC_OK_BM) != CC2500_LQI_CRC_OK_BM)
    {
        return(RX_CRC_MISMATCH);
    }
    return(RX_OK);
}
