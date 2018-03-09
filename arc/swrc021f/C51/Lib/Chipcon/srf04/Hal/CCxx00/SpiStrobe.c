/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements writing a strobe command to the the CCxxx0.           *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SpiStrobe.c                                                                *
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
//  void halSpiStrobe(BYTE strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      BYTE strobe
//          Strobe command
//-------------------------------------------------------------------------------------------------------
void halSpiStrobe(BYTE strobe) {
    NSSMD0 = 0;
    while (P0_1);
    SPI0DAT = strobe;
    SPI_WAIT();
    NSSMD0 = 1;
}// halSpiStrobe




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: SpiStrobe.c,v $
 * Revision 1.3  2005/03/18 14:42:18  sna
 * Added while (P0_1);  after NSSMD0 = 0;
 * When CSn goes low, the MCU must wait until
 * the SO pin from CC2500 goes low before
 * starting to transfer the header byte. This
 * indicates that the voltage regulator has
 * stabilized and the crystal is running.
 *
 * Revision 1.2  2005/02/07 14:45:44  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
