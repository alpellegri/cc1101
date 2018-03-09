/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains all functions related to packet reception in the        *
 *      ***   +             Link1 example                                                              *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Rx.c                                                                       *
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
#include <Link1.h>




//-------------------------------------------------------------------------------------------------------
//  Global Variables
PKT_DATA xdata pktData;
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void pktStartRx(UINT16 timeout)
//
//  DESCRIPTION:
//      This function should be called every time the radio shall enter RX mode.
//
//  ARGUMENTS:
//      UINT16 timeout
//          If the radio shall go back to IDLE after a given time, timeout =! 0.
//          RX Timeout [ms] = timeout * 200 us
//          Min RX Timeout: 0.2 ms
//          Max RX Timeout: 13107 ms
//          timeout = 0 => No timeout. Radio will stay in RX until a packet is received,
//-------------------------------------------------------------------------------------------------------
void pktStartRx(UINT16 timeout) {
    if (timeout) {
        pktData.rxTimeout = timeout;
        pktData.rxTimeoutActive = TRUE;
        pktData.rxTimeoutReached = FALSE;
    }
    ENABLE_GLOBAL_INT(INT_OFF);
    halSpiStrobe(CCxxx0_SRX);
    ENABLE_GLOBAL_INT(INT_ON);
}// pktStartRx




//-------------------------------------------------------------------------------------------------------
//  void pktRxHandler(void)
//
//  DESCRIPTION:
//      This function is called every time a timer1 interrupt occurs (every 200 us). The function starts
//      with timeout handling and continues by getting the status byte. A counter, 
//      pktData.rxTimeout, has an initial value timeout (see pktStartRx) and is decremented each
//      time this function is called until it reaches 0 => strobe IDLE (RX mode is terminated).
//      Every time the status byte indicates that there are available bytes in the RX FIFO, bytes are read
//      from the RX FIFO and written to rxBuffer. This is done until the whole packet is received
//      (pktData.pktReceived = TRUE). Due to the FIFO bug, the RX FIFO is not emptied before the last 
//      byte has been received (see Discrepancy Note). If the status byte indicates that there has been an
//      RX FIFO overflow, or the timeout has been reached, the RX FIFO is being flushed. Please see the 
//      CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual for a flow 
//      chart describing this function.
//-------------------------------------------------------------------------------------------------------
void pktRxHandler(void) {
    BOOL error = FALSE;
    BYTE spiRxStatus;
    UINT16 bytesInFifo;

    // Timeout handling
    if (pktData.rxTimeoutActive) {
        if (pktData.rxTimeout)
            pktData.rxTimeout--;
        else {
            halSpiStrobe(CCxxx0_SIDLE);
            pktData.rxTimeoutActive = FALSE;
            pktData.rxTimeoutReached = TRUE;
        }
    }

    // Which state?
    spiRxStatus = spiGetRxStatus();

    if ((spiRxStatus & CC2500_STATE_BM) == CC2500_STATE_RX_OVERFLOW_BM)
        error = TRUE;
    
    // Handle RX FIFO overflow/RX turnoff timeout
    if (error | pktData.rxTimeoutReached) {
        pktData.rxBytesLeft = 0;
        halSpiStrobe(CCxxx0_SFRX);
    } else {

        // Send the RX FIFO address and get the number of bytes in the FIFO
        CSn = LOW;

        // Wait for SO to go low (voltage regulator has stabilized and the crystal is running
        while (P_SO);

        // Burst access to RX FIFO
        SPI0DAT = CCxxx0_RXFIFO | READ_BURST;
        SPI_WAIT();

        // Read back status byte on SO line
        spiRxStatus = SPI0DAT;

        // If there's anything in the RX FIFO....
        if (bytesInFifo = spiRxStatus & CC2500_FIFO_BYTES_AVAILABLE_BM) {

            // Start by getting the packet length
			if ((pktData.lengthByteRead == FALSE) && (bytesInFifo != 1)) {
                SPI0DAT = 0;
                SPI_WAIT();

                pktData.pRxBuffer[0] = SPI0DAT;
                pktData.rxBytesLeft = pktData.pRxBuffer[0] + 2; // Packet Length + 2 status bytes
                bytesInFifo--;
                pktData.rxPosition = 1;
                pktData.lengthByteRead = TRUE;
            }

            // Make sure that the RX FIFO will not be emptied (See the CC1100 or 2500 Errata Note)
            if ((bytesInFifo) && (bytesInFifo != pktData.rxBytesLeft))
                bytesInFifo--;

            // Update how many bytes are left to be received
            pktData.rxBytesLeft -= bytesInFifo;

            // Read from RX FIFO and store the data in rxBuffer
            while (bytesInFifo--) {
                SPI0DAT = 0;
                SPI_WAIT();
                pktData.pRxBuffer[pktData.rxPosition++] = SPI0DAT;
            }
            CSn = HIGH;

            // Done?
            if ((!pktData.rxBytesLeft) && (pktData.lengthByteRead)) {
                pktData.pktReceived = TRUE;
                pktData.lengthByteRead = FALSE;
            }
        }
        CSn = HIGH;

    }
} // pktRxHandler




//-------------------------------------------------------------------------------------------------------
//  BYTE spiGetRxStatus(void)
//
//  DESCRIPTION:
//      This function transmits a No Operation Strobe (SNOP) with the read bit set to get the status of 
//      the radio. 
//
//      Status byte:
//
//      --------------------------------------------------------------------------------
//      |          |            |                                                      |
//      | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO |
//      |          |            |                                                      |
//      --------------------------------------------------------------------------------
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
BYTE spiGetRxStatus(void) {
    CSn = LOW;

    // Wait for SO to go low (voltage regulator has stabilized and the crystal is running
    while (P_SO);

    // Strobe SNOP with the read byte set to get info on current state and number of bits in the RX FIFO 
    SPI0DAT = CCxxx0_SNOP | 0x80;
    SPI_WAIT();
    CSn = HIGH;
    return SPI0DAT;
} // spiGetRxStatus




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: Rx.c,v $
 * Revision 1.6  2006/04/25 14:55:28  a0190596
 * no message
 *
 * Revision 1.5  2006/03/27 15:09:23  a0190596
 * Disable global interrupt before strobing SRX
 *
 * Revision 1.4  2005/10/25 12:07:04  sna
 * Changed test value in if statement to make it easier to read the code
 *
 * Revision 1.3  2005/07/11 12:12:11  sna
 * no message
 *
 * Revision 1.2  2005/06/14 11:08:29  sna
 * no message
 *
 * Revision 1.1  2005/04/20 12:42:07  sna
 * Initial version in CVS
 *
 *
 *
 ******************************************************************************************************/