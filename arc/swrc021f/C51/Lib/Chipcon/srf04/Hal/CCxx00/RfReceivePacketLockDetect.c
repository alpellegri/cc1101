/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function checks for lock and receives a packet                        *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfReceivePacketLockDetect.c                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:        See end of file                                                            *
 ******************************************************************************************************/
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\regssrf04.h>



//-------------------------------------------------------------------------------------------------------
// Defines
#define CRC_OK              0x80  
#define GDO0_PIN            P0_6
#define RSSI                0
#define LQI                 1
#define BYTES_IN_RXFIFO     0x7F        
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacketLockDetect(BYTE *rxBuffer, UINT8 *length, *pLock)
//
//  DESCRIPTION: 
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      GD02 must be configured to be asserted when lock is detected
//      => halSpiWriteReg(CCxxx0_IOCFG2, 0x0A);
//      Also, APPEND_STATUS in the PKTCTRL1 register must be enabled. If the PLL is not locked within
//      ~920 us the radio is forced to IDLE state and the RX FIFO is being flushed.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//      After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//      are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//      FIFO is flushed due to address filtering, CRC filtering, or packet length filtering. 
//  
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//
//      BOOL *pLock
//          Pointer to a variable that is set in an ISR when the Lock Detector is asserted.
//          
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK, 
//                      or 
//                  no packet was put in the RX FIFO due to filtering
//                      or
//                  the PLL did not lock 
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacketLockDetect(BYTE *rxBuffer, UINT8 *length, BOOL *pLock) {
    BYTE status[2];
    UINT8 packetLength;
    UINT16 lockTimeout = 1300;

    ENABLE_GLOBAL_INT(INT_OFF);
    SETUP_GDO2_INT(EDGE, HIGH);
    INT_SETFLAG(INUM_EXTERNAL1, INT_CLR);
    ENABLE_GLOBAL_INT(INT_ON);

    halSpiStrobe(CCxxx0_SRX);

    while (((*pLock == FALSE)) && (lockTimeout--));

    INT_ENABLE(INUM_EXTERNAL1, INT_OFF);

    if ((*pLock == TRUE)) {

        // Wait for GDO0 to be set -> sync received
        while (!GDO0_PIN);

        // Wait for GDO0 to be cleared -> end of packet
        while (GDO0_PIN);

        *pLock = FALSE;

        // This status register is safe to read since it will not be updated after
        // the packet has been received (See the CC11xx and 25xx Errata Note)
        if ((halSpiReadStatus(CCxxx0_RXBYTES) & BYTES_IN_RXFIFO)) {

            // Read length byte
            packetLength = halSpiReadReg(CCxxx0_RXFIFO);
    
            // Read data from RX FIFO and store in rxBuffer
            if (packetLength <= *length) {
                halSpiReadBurstReg(CCxxx0_RXFIFO, rxBuffer, packetLength); 
                *length = packetLength;
        
                // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
                halSpiReadBurstReg(CCxxx0_RXFIFO, status, 2); 
        
                // MSB of LQI is the CRC_OK bit
                return (status[LQI] & CRC_OK);
            } else {
                *length = packetLength;

                // Flush RX FIFO
                halSpiStrobe(CCxxx0_SFRX);
                return FALSE;
            }
        } else
            return FALSE;
    } else {

        *pLock = FALSE;

        halSpiStrobe(CCxxx0_SIDLE);
        halSpiStrobe(CCxxx0_SFRX);
        return FALSE;
    }
}// halRfReceivePacketLockDetect




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: RfReceivePacketLockDetect.c,v $
 * Revision 1.2  2006/03/17 10:02:49  a0190596
 * Comments added regarding reading the RXBYTES register
 *
 * Revision 1.1  2005/12/20 12:09:37  sna
 * Initial version in CVS.
 *
 *
 *
 ******************************************************************************************************/