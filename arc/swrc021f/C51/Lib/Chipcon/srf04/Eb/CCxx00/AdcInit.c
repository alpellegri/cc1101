/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements functionality for reading the potmeter                *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     AdcInit.c                                                                  *
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
#include <Chipcon\srf04\ebsrf04.h>




//-------------------------------------------------------------------------------------------------------
//  void ebAdcInit(UINT8 adcInput)
//
//  DESCRIPTION:
//      Function used to initialize ADC before using the ebGetJoystickPosition or ebReadPotentiometer
//      functions.
//
//  ARGUMENTS:
//      UINT8 adcInput
//          ADC_JOY
//          ADC_POT
//          ADC_AUDIO
//-------------------------------------------------------------------------------------------------------
void ebAdcInit(UINT8 adcInput) {
   
    // Setup the ADC: Low-power Track Mode (ADC0 conversion initiated on every write of ‘1’ to AD0BUSY)
    // SAR Conversion Clock = 3000 kHz and data in ADC0H:ADC0L registers are left-justified.
    ADC0CF = BM_AD0SC2 | BM_AD0SC1 | BM_AD0SC0 | BM_AD0LJST;    
    ADC0CN = BM_AD0TM;
    ADC_ENABLE();
    REF0CN = BM_REFSL;              // VDD used as voltage reference
    // Select the correct signal as ADC input
    AMX0P = adcInput;
    AMX0N = 0x1F;                   // ADC in Single-Ended Mode
}// ebAdcInit




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: AdcInit.c,v $
 * Revision 1.2  2005/02/07 14:43:19  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
