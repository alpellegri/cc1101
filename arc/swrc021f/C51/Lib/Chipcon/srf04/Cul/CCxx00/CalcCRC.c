/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function calculates RC-16/CCITT.                                      *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     CalcCRC.c                                                                  *
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
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\culsrf04.h>




//-------------------------------------------------------------------------------------------------------
//  Defines
#define CRC16_POLY          0x8005
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  UINT16 culCalcCRC(BYTE crcData, UINT16 crcReg)
//
//  DESCRIPTION:
//      A CRC-16/CCITT implementation optimized for small code size. The function should be called once 
//      for each byte in the data the CRC is to be performed on. For the invocation on the first byte
//      the value CRC_INIT should be given for _crcReg_. The value returned is the CRC-16 of the data 
//      supplied so far. This CRC-value should be added at the end of the data to facilitate a later 
//      CRC check. During checking the check should be performed on all the data AND the CRC-16
//      value appended to it. The data is intact if the value returned is 0. 
//
//  ARGUMENTS: 
//      BYTE crcData
//          The data to perform the CRC-16 operation on.
//      UINT16 crcReg
//          The current value of the CRC register. For the first byte the value CRC16_INIT should be 
//          supplied. For each additional byte the value returned for the last invocation should be 
//          supplied.
//
//  RETURN VALUE:
//      The updated value of the CRC16 register. This corresponds to the CRC-16 of the data supplied 
//      so far. During CRC checking, after working through all the data and the appended CRC-16 value, 
//      the value will be 0 if the data is intact.
//-------------------------------------------------------------------------------------------------------
UINT16 culCalcCRC(BYTE crcData, UINT16 crcReg) {
    UINT8 i;

    for (i = 0; i < 8; i++) {
        if (((crcReg & 0x8000) >> 8) ^ (crcData & 0x80))
            crcReg = (crcReg << 1) ^ CRC16_POLY;
        else
            crcReg = (crcReg << 1);

        crcData <<= 1;
    }
    return crcReg;
}// culCalcCRC




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: CalcCRC.c,v $
 * Revision 1.1  2005/11/09 09:33:07  sna
 * Initial version in CVS
 *
 *
 *
 ******************************************************************************************************/