/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains macros and function that are useful when                *
 *      ***   +             accessing peripherals on the SmartRF04EB using the c8051f320               *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     culsrf04.h                                                                 *
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
#ifndef CULSRF04_H
#define CULSRF04_H
#include <Chipcon\srf04\common.h>




//-------------------------------------------------------------------------------------------------------
// Defines
#define CRC_INIT        0xFFFF
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  UINT16 culCalcCRC(BYTE crcData, UINT16 crcReg)
//
//  DESCRIPTION:
//      A CRC-16/CCITT implementation optimized for small code size. The function should be called once 
//      for each byte in the data the CRC is to be performed on. For the invocation on the first byte
//      the value CRC16_INIT should be given for _crcReg_. The value returned is the CRC-16 of the data 
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
UINT16 culCalcCRC(BYTE crcData, UINT16 crcReg);




//-------------------------------------------------------------------------------------------------------
//  void culSyncSearch(UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0)
//
//  DESCRIPTION:
//      This function search for a 4 bytes sync word. 
//
//  ARGUMENTS:
//      UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0
//          The sunc word that is being searched for. sync3 is the most significant byte.
//-------------------------------------------------------------------------------------------------------
void culSyncSearch(UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0);



/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: culsrf04.h,v $
 * Revision 1.1  2005/11/09 10:03:15  sna
 * Initial version in CVS.
 *
 *
 *
 ******************************************************************************************************/




#endif// CULSRF04_H

