/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function gets the value of a single specified CCxxx0 register.        *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SpiReadReg.c                                                               *
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
//  BYTE halSpiReadReg(BYTE addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
BYTE halSpiReadReg(BYTE addr) {
    UINT8 x;
    NSSMD0 = 0;
    while (P0_1);
    SPI0DAT = (addr | READ_SINGLE);
    SPI_WAIT();
    SPI0DAT = 0;
    SPI_WAIT();
    x = SPI0DAT;
    NSSMD0 = 1;
    return x;
}// halSpiReadReg




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SpiReadReg.c,v $
 * Revision 1.4  2007/06/22 13:32:03  a0190596
 * no message
 *
 * Revision 1.3  2005/03/18 14:42:05  sna
 * Added while (P0_1);  after NSSMD0 = 0;
 * When CSn goes low, the MCU must wait until
 * the SO pin from CC2500 goes low before
 * starting to transfer the header byte. This
 * indicates that the crystal is running.
 *
 * Revision 1.2  2005/02/07 14:45:31  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/

