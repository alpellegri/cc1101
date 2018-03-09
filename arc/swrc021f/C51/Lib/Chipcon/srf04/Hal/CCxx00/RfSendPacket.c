/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function transmits a packet                                           *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfSendPacket.c                                                             *
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
#define GDO0_PIN        P0_6            
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacket(BYTE *txBuffer, UINT8 size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to 63 bytes.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.  
//      
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containing the data that are going to be transmitted
//
//      UINT8 size
//          The size of the txBuffer
//-------------------------------------------------------------------------------------------------------
void halRfSendPacket(BYTE *txBuffer, UINT8 size) {

    halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);

    halSpiStrobe(CCxxx0_STX);

    // Wait for GDO0 to be set -> sync transmitted
    while (!GDO0_PIN);

    // Wait for GDO0 to be cleared -> end of packet
    while (GDO0_PIN);

}// halRfSendPacket




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: RfSendPacket.c,v $
 * Revision 1.6  2005/10/25 09:50:48  sna
 * Corrected typing error
 *
 * Revision 1.5  2005/07/11 12:28:16  sna
 * no message
 *
 * Revision 1.4  2005/06/30 08:32:58  sna
 * Uses GDO0 instead of GDO2 since CCxx50 does not have GDO2
 *
 * Revision 1.3  2005/06/14 11:06:26  sna
 * no message
 *
 * Revision 1.2  2005/02/07 14:44:56  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
