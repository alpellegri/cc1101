/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function receives a packet                                            *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfReceivePacket.c                                                          *
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
//  BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION: 
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      Also, APPEND_STATUS in the PKTCTRL1 register must be enabled.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//      After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//      are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//      FIFO is flushed due to address filtering, CRC filtering, or packet length filtering. 
//  
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//          
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to filtering)
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length) {
    BYTE status[2];
    UINT8 packetLength;

    halSpiStrobe(CCxxx0_SRX);

    // Wait for GDO0 to be set -> sync received
    while (!GDO0_PIN);

    // Wait for GDO0 to be cleared -> end of packet
    while (GDO0_PIN);

    // This status register is safe to read since it will not be updated after
    // the packet has been received (See the CC1100 and 2500 Errata Note)
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

            // Make sure that the radio is in IDLE state before flushing the FIFO
            // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state at this point) 
            halSpiStrobe(CCxxx0_SIDLE);

            // Flush RX FIFO
            halSpiStrobe(CCxxx0_SFRX);
            return FALSE;
        }
    } else
        return FALSE;
}// halRfReceivePacket




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: RfReceivePacket.c,v $
 * Revision 1.9  2006/10/06 13:32:48  a0190596
 * no message
 *
 * Revision 1.8  2006/03/17 10:03:26  a0190596
 * Comments added regarding reading the RXBYTES register
 *
 * Added SIDLE strobe
 *
 * Revision 1.7  2005/11/09 09:38:35  sna
 * Function will return FALSE when no data was put in the RX FIFO due to filtering
 * (previously no return value was defined for this case)
 *
 * Revision 1.6  2005/10/25 09:50:26  sna
 * Corrected typing error
 *
 * Revision 1.5  2005/09/29 06:54:27  sna
 * #define BYTES_IN_RXFIFO     0x7F is changed from
 * #define BYTES_IN_RXFIFO     0xEF
 *
 * Revision 1.4  2005/07/11 12:27:49  sna
 * CRC_OK is checked in the FIFO instead of in PKTSTATUS.
 * After packet received is detected, RXBYTES is read to make sure that the FIFO
 * has not been flushed due to address filtering, CRC filtering or packet length
 * filtering
 *
 * Revision 1.3  2005/06/30 08:32:42  sna
 * Uses GDO0 instead of GDO2 since CCxx50 does not have GDO2
 * Check CRC using status byte in FIFO instead of using PKTSTATUS
 *
 * Revision 1.2  2005/02/07 14:44:48  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/