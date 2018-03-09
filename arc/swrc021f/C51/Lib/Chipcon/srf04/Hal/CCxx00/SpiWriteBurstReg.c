/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements a function for setting multiple                       *
 *      ***   +             CCxxx0 registers, using SPI burst access.                                  *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SpiWriteBurstReg.c                                                         *
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
//  void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.   
//-------------------------------------------------------------------------------------------------------
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count) {
    UINT8 i;
    NSSMD0 = 0;
    while (P0_1);
    SPI0DAT = addr | WRITE_BURST;
    SPI_WAIT();
    for (i = 0; i < count; i++) {
        SPI0DAT = buffer[i];
        SPI_WAIT();
    }
    NSSMD0 = 1;
}// halSpiWriteBurstReg




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SpiWriteBurstReg.c,v $
 * Revision 1.3  2005/03/18 14:42:29  sna
 * Added while (P0_1);  after NSSMD0 = 0;
 * When CSn goes low, the MCU must wait until
 * the SO pin from CC2500 goes low before
 * starting to transfer the header byte. This
 * indicates that the voltage regulator has
 * stabilized and the crystal is running.
 *
 * Revision 1.2  2005/02/07 14:45:57  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
