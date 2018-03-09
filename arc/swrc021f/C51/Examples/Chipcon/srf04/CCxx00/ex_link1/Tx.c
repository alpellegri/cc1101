/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains all functions related to packet transmission in the     *
 *      ***   +             Link1 example                                                              *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Tx.c                                                                       *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 *                                                                                                     *
 ******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>
#include <stdlib.h>
#include <Link1.h>




//-------------------------------------------------------------------------------------------------------
//  void createDataPacket(void) 
//
//  DESCRIPTION: 
//      This function is called before a packet is going to be transmitted.
//      Packet format:
//
//  |----------------------------------------------------------------------------------------------------
//  |              |               |             |             |            |             |             |
//  | Length field | Address Field | Random Data | Random Data |............| Random Data | Random Data |
//  |              |               |             |             |            |             |             |
//  |----------------------------------------------------------------------------------------------------
//          ^             ^               ^                                                      ^
//          |             |               |                                                      |
//     txBuffer[0]    txBuffer[1]    txBuffer[2]                          txBuffer[menuData.packetLength]
//-------------------------------------------------------------------------------------------------------
void createDataPacket(void) {
    UINT16 xdata i;

    txBuffer[0] = menuData.packetLength;
    txBuffer[1] = ADDR;
    for (i = 2; i <= menuData.packetLength; i++)
        txBuffer[i] = rand();    
} //createDataPacket




//-------------------------------------------------------------------------------------------------------
//  void pktStartTx(UINT16 timeout)
//
//  DESCRIPTION:
//      This function should be called every time the radio shall enter TX mode.
//      The data that shall be transmitted must be placed in txBuffer before this function is being
//      called. The function also updates some variables related to the TX timeout. 
//
//  ARGUMENTS:
//      UINT16 timeout
//          The timeout variable determine the delay before the radio enters TX mode.
//          TX Timeout [ms] = timeout * 200 us
//          Min TX Timeout: 0 ms
//          Max TX Timeout: 13107 ms
//-------------------------------------------------------------------------------------------------------
void pktStartTx(UINT16 timeout) {
    pktData.txPosition = 1; // Skip the length byte, which is handled separately
    pktData.txBytesLeft = pktData.pTxBuffer[0];
    pktData.txStartupTimeout = timeout;
    pktData.txStartupTimeoutActive = TRUE;
}// pktStartTx




//-------------------------------------------------------------------------------------------------------
//  BYTE pktTxHandler(void)
//
//  DESCRIPTION:
//      This function is called every time a timer1 interrupt occurs (every 200 us). The function starts
//      with timeout handling and continues by getting the status byte. A counter, 
//      pktData.txStartupTimeout, has an initial value timeout (see pktStartTx) and is decremented each
//      time this function is called until it reaches 0 => strobe TX
//      Every time the status byte indicates that there is free space in the TX FIFO, bytes are taken from
//      txBuffer and written to the TX FIFO until the whole packet is written (pktData.txBytesLeft = 0 
//      and pktData.pktTransmitted = TRUE) or the TXFIFO has underflowed. When the status byte indicates 
//      that the radio has gone back to IDLE state, the pktData.txInProgress flag is cleared. Please see 
//      the CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual for a 
//      flow chart describing this function.
//-------------------------------------------------------------------------------------------------------
void pktTxHandler(void) {
    UINT8 freeSpaceInFifo;
    BYTE spiTxStatus;

    // Handle startup timeout (when active)
    if (pktData.txStartupTimeoutActive) {

        // Start TX
        if (pktData.txStartupTimeout == 0) {
            pktData.txStartupTimeoutActive = FALSE;

            // Enter TX mode and write the length byte
            pktData.txInProgress = TRUE;
            halSpiStrobe(CCxxx0_STX);
            halSpiWriteReg(CCxxx0_TXFIFO, pktData.txBytesLeft);

        // Normal countdown
        } else {
            pktData.txStartupTimeout--;
            return;
        }
    }

    // Which state?
    spiTxStatus = spiGetTxStatus();

    switch (spiTxStatus & CC2500_STATE_BM) {
        case CC2500_STATE_TX_BM:

            // If there's anything to transfer..
            if (freeSpaceInFifo = MIN(PKT_MAX_BYTES_PER_TRANSFER, MIN(pktData.txBytesLeft, spiTxStatus & CC2500_FIFO_BYTES_AVAILABLE_BM))) {
                pktData.txBytesLeft -= freeSpaceInFifo;

                CSn = LOW;
            
                // Wait for SO to go low (voltage regulator has stabilized and the crystal is running
                while (P_SO);

                // Burst access to TX FIFO  
                SPI0DAT = CCxxx0_TXFIFO | WRITE_BURST;
                SPI_WAIT();
                
                while (freeSpaceInFifo--) {
                    SPI0DAT = pktData.pTxBuffer[pktData.txPosition++];
                    SPI_WAIT();
                }
                
                CSn = HIGH;

                // Notify the application if all bytes in the packet has been written to the TX FIFO
                if (!pktData.txBytesLeft)
                    pktData.pktTransmitted = TRUE;
            }
            break;

        case CC2500_STATE_TX_UNDERFLOW_BM:
            halSpiStrobe(CCxxx0_SFTX);  // Flush the TX FIFO
            pktData.txUnderflowFlag = TRUE;

            // No break here!
    
        default:
            if (pktData.txInProgress) {
                if ((spiTxStatus & CC2500_STATE_BM) == CC2500_STATE_IDLE_BM) {
    	           pktData.txInProgress = FALSE;
                }
            }
        break;
    }
} // pktTxHandler




//-------------------------------------------------------------------------------------------------------
//  BYTE spiGetTxStatus(void) 
//
//  DESCRIPTION:
//      This function transmits a No Operation Strobe (SNOP) to get the status of the radio. 
//
//      Status byte:
//
//      ---------------------------------------------------------------------------
//      |          |            |                                                 |
//      | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
//      |          |            |                                                 |
//      ---------------------------------------------------------------------------
//
//      STATE[2:0]: 
//
//      Value | State
//      --------------------------
//      000   | Idle
//      001   | RX
//      010   | TX
//      011   | FSTXON
//      100   | CALIBRATE
//      101   | SETTLING
//      110   | RXFIFO_OVERFLOW
//      111   | TX_FIFO_UNDERFLOW
//-------------------------------------------------------------------------------------------------------
BYTE spiGetTxStatus(void) {
    CSn = LOW;

    // Wait for SO to go low (voltage regulator has stabilized and the crystal is running
    while (P_SO);

    // Strobe SNOP to get info on current state and number of bits in the TX FIFO
    SPI0DAT = CCxxx0_SNOP;
    SPI_WAIT();
    CSn = HIGH;
    return SPI0DAT;
} // spiGetTxStatus




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: Tx.c,v $
 * Revision 1.6  2007/01/25 12:14:35  a0190596
 * no message
 *
 * Revision 1.5  2007/01/23 15:47:58  a0190596
 * The default case in the pktTxHandler in Tx.c has been changed to properly handle underflow of the TX FIFO.
 *
 * Revision 1.4  2006/04/25 14:54:30  a0190596
 * no message
 *
 * Revision 1.3  2006/03/31 12:45:23  a0190596
 * Typing error corrected
 *
 * Revision 1.2  2005/07/11 12:12:31  sna
 * no message
 *
 * Revision 1.1  2005/04/20 12:42:21  sna
 * Initial version in CVS
 *
 *
 *
 ******************************************************************************************************/