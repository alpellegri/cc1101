/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function checks for lock and transmits a packet                       *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfSendPacketLockDetect.c                                                   *
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
//  BOOL halRfSendPacketLockDetect(BYTE *txBuffer, UINT8 size, BOOL *pLock)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to 63 bytes.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      GD02 must be configured to be asserted when lock is detected
//      => halSpiWriteReg(CCxxx0_IOCFG2, 0x0A);
//      If the PLL is not locked within ~920 us, the radio is forced back in IDLE state and no packet is
//      being sent.If the PLL is locked within 920 us, TX mode is entered.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.  
//      
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containing the data that are going to be transmitted
//
//      UINT8 size
//          The size of the txBuffer
//
//      BOOL *pLock
//          Pointer to a variable that is set in an ISR when the Lock Detector is asserted.
//
//  RETURN VALUE:
//      BOOL
//          TRUE; A packet has been sent
//          FALSE: Radio is forced to IDLE state due to lock failure. No packet is being sent.
//-------------------------------------------------------------------------------------------------------
BOOL halRfSendPacketLockDetect(BYTE *txBuffer, UINT8 size, BOOL *pLock) {

    UINT16 xdata lockTimeout = 1300;    // This will give a timeout of about 920 us.

    ENABLE_GLOBAL_INT(INT_OFF);
    SETUP_GDO2_INT(EDGE, HIGH);
    
    halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, size);


    INT_SETFLAG(INUM_EXTERNAL1, INT_CLR);
    ENABLE_GLOBAL_INT(INT_ON);

    halSpiStrobe(CCxxx0_SFSTXON);

    // Wait max 920 us for the PLL to lock
    while (((*pLock) == FALSE) && (lockTimeout--));

    INT_ENABLE(INUM_EXTERNAL1, INT_OFF);

    if ((*pLock) == TRUE) {     // PLL is in lock
    
        halSpiStrobe(CCxxx0_STX);
    
        // Wait for GDO0 to be set -> sync transmitted
        while (!GDO0_PIN);

        // Wait for GDO0 to be cleared -> end of packet
        while (GDO0_PIN);
        
        *pLock = FALSE;

        return TRUE;
    } else {                    // PLL is not in lock. Return to IDLE state and flush the TX FIFO
        halSpiStrobe(CCxxx0_SIDLE);
        halSpiStrobe(CCxxx0_SFTX);
        *pLock = FALSE;
        return FALSE;
    }
}// halRfSendPacketLockDetect




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: RfSendPacketLockDetect.c,v $
 * Revision 1.1  2005/12/20 12:09:37  sna
 * Initial version in CVS.
 *
 *
 *
 ******************************************************************************************************/