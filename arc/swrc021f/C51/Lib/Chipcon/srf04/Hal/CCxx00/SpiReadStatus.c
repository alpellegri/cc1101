/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function reads a CCxxx0 status register.                              *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SpiReadStatus.c                                                            *
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
//  BYTE halSpiReadStatus(BYTE addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 status register.
//-------------------------------------------------------------------------------------------------------
BYTE halSpiReadStatus(BYTE addr) {
    UINT8 x;
    NSSMD0 = 0;
    while (P0_1);
    SPI0DAT = (addr | READ_BURST);
    SPI_WAIT();
    SPI0DAT = 0;
    SPI_WAIT();
    x = SPI0DAT;
    NSSMD0 = 1;
    return x;
}// halSpiReadStatus




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SpiReadStatus.c,v $
 * Revision 1.3  2005/03/18 14:42:12  sna
 * Added while (P0_1);  after NSSMD0 = 0;
 * When CSn goes low, the MCU must wait until
 * the SO pin from CC2500 goes low before
 * starting to transfer the header byte. This
 * indicates that the voltage regulator has
 * stabilized and the crystal is running.
 *
 * Revision 1.2  2005/02/07 14:45:38  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
