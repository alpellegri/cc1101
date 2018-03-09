/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements writing configuration values to the CC2500.           *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfWriteRfSettings.c                                                        *
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
//  void RfWriteRfSettings(RF_SETTINGS *pRfSettings)
//
//  DESCRIPTION:
//      This function is used to configure the CCxxx0 based on a given rf setting
//
//  ARGUMENTS:
//      RF_SETTINGS *pRfSettings
//          Pointer to a struct containing rf register settings
//-------------------------------------------------------------------------------------------------------
void halRfWriteRfSettings(RF_SETTINGS *pRfSettings) {

    // Write register settings
    halSpiWriteReg(CCxxx0_FSCTRL1,  pRfSettings->FSCTRL1);
    halSpiWriteReg(CCxxx0_FSCTRL0,  pRfSettings->FSCTRL0);
    halSpiWriteReg(CCxxx0_FREQ2,    pRfSettings->FREQ2);
    halSpiWriteReg(CCxxx0_FREQ1,    pRfSettings->FREQ1);
    halSpiWriteReg(CCxxx0_FREQ0,    pRfSettings->FREQ0);
    halSpiWriteReg(CCxxx0_MDMCFG4,  pRfSettings->MDMCFG4);
    halSpiWriteReg(CCxxx0_MDMCFG3,  pRfSettings->MDMCFG3);
    halSpiWriteReg(CCxxx0_MDMCFG2,  pRfSettings->MDMCFG2);
    halSpiWriteReg(CCxxx0_MDMCFG1,  pRfSettings->MDMCFG1);
    halSpiWriteReg(CCxxx0_MDMCFG0,  pRfSettings->MDMCFG0);
    halSpiWriteReg(CCxxx0_CHANNR,   pRfSettings->CHANNR);
    halSpiWriteReg(CCxxx0_DEVIATN,  pRfSettings->DEVIATN);
    halSpiWriteReg(CCxxx0_FREND1,   pRfSettings->FREND1);
    halSpiWriteReg(CCxxx0_FREND0,   pRfSettings->FREND0);
    halSpiWriteReg(CCxxx0_MCSM0 ,   pRfSettings->MCSM0 );
    halSpiWriteReg(CCxxx0_FOCCFG,   pRfSettings->FOCCFG);
    halSpiWriteReg(CCxxx0_BSCFG,    pRfSettings->BSCFG);
    halSpiWriteReg(CCxxx0_AGCCTRL2, pRfSettings->AGCCTRL2);
	halSpiWriteReg(CCxxx0_AGCCTRL1, pRfSettings->AGCCTRL1);
    halSpiWriteReg(CCxxx0_AGCCTRL0, pRfSettings->AGCCTRL0);
    halSpiWriteReg(CCxxx0_FSCAL3,   pRfSettings->FSCAL3);
    halSpiWriteReg(CCxxx0_FSCAL2,   pRfSettings->FSCAL2);
	halSpiWriteReg(CCxxx0_FSCAL1,   pRfSettings->FSCAL1);
    halSpiWriteReg(CCxxx0_FSCAL0,   pRfSettings->FSCAL0);
    halSpiWriteReg(CCxxx0_FSTEST,   pRfSettings->FSTEST);
    halSpiWriteReg(CCxxx0_TEST2,    pRfSettings->TEST2);
    halSpiWriteReg(CCxxx0_TEST1,    pRfSettings->TEST1);
    halSpiWriteReg(CCxxx0_TEST0,    pRfSettings->TEST0);
    halSpiWriteReg(CCxxx0_FIFOTHR,  pRfSettings->FIFOTHR);
    halSpiWriteReg(CCxxx0_IOCFG2,   pRfSettings->IOCFG2);
    halSpiWriteReg(CCxxx0_IOCFG0,   pRfSettings->IOCFG0);    
    halSpiWriteReg(CCxxx0_PKTCTRL1, pRfSettings->PKTCTRL1);
    halSpiWriteReg(CCxxx0_PKTCTRL0, pRfSettings->PKTCTRL0);
    halSpiWriteReg(CCxxx0_ADDR,     pRfSettings->ADDR);
    halSpiWriteReg(CCxxx0_PKTLEN,   pRfSettings->PKTLEN);
}// halRfWriteRfSettings



   

/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: RfWriteRfSettings.c,v $
 * Revision 1.9  2007/11/23 09:14:13  a0190596
 * FIFOTHR added
 *
 * Revision 1.8  2005/10/25 09:52:05  sna
 * Added AGCCTRL1 and FSCAL1.
 *
 * Revision 1.7  2005/06/30 08:34:22  sna
 * Fixed faulty configuration of GDOx pins
 *
 * Revision 1.6  2005/06/14 11:06:13  sna
 * no message
 *
 * Revision 1.5  2005/05/26 15:16:26  sna
 * Updated according to new RF_SETTINGS struct
 * Removed error in register name
 *
 * Revision 1.4  2005/05/26 15:13:28  sna
 * Updated according to new RF_SETTINGS struct
 *
 * Revision 1.3  2005/04/11 13:26:27  sna
 * Updated the RF_SETTINGS struct according to SmartRF Studio 6.0.4
 *
 * Revision 1.2  2005/02/07 14:45:04  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
