/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements a function for getting multiple                       *
 *      ***   +             CCxxx0 registers, using SPI burst access.                                  *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SpiReadBurstReg.c                                                          *
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
//  void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count) {
    UINT8 i;
    NSSMD0 = 0;
    while (P0_1);
    SPI0DAT = (addr | READ_BURST);
    SPI_WAIT();  
    for (i = 0; i < count; i++) {
        SPI0DAT = 0;
        SPI_WAIT();
        buffer[i] = SPI0DAT;
    }
    NSSMD0 = 1;
}// halSpiReadBurstReg




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SpiReadBurstReg.c,v $
 * Revision 1.4  2005/11/09 09:39:09  sna
 * no message
 *
 * Revision 1.3  2005/03/18 14:41:56  sna
 * Added while (P0_1);  after NSSMD0 = 0;
 * When CSn goes low, the MCU must wait until
 * the SO pin from CC2500 goes low before
 * starting to transfer the header byte. This
 * indicates that the voltage regulator has
 * stabilized and the crystal is running.
 *
 * Revision 1.2  2005/02/07 14:45:24  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
