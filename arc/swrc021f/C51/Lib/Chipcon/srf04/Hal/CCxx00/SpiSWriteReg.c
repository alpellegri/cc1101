/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements a function for setting                                *
 *      ***   +             a single register in CCxxx0, using SPI.                                    *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SpiWriteReg.c                                                              *
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
//  void halSpiWriteReg(BYTE addr, BYTE value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      BYTE addr
//          Address of a specific CCxxx0 register to accessed.
//      BYTE value
//          Value to be written to the specified CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
void halSpiWriteReg(BYTE addr, BYTE value) {
    NSSMD0 = 0;
    while (P0_1);
    SPI0DAT = addr;
    SPI_WAIT();
    SPI0DAT = value;
    SPI_WAIT();
    NSSMD0 = 1;
}// halSpiWriteReg




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SpiSWriteReg.c,v $
 * Revision 1.3  2005/03/18 14:42:24  sna
 * Added while (P0_1);  after NSSMD0 = 0;
 * When CSn goes low, the MCU must wait until
 * the SO pin from CC2500 goes low before
 * starting to transfer the header byte. This
 * indicates that the voltage regulator has
 * stabilized and the crystal is running.
 *
 * Revision 1.2  2005/02/07 14:45:51  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
