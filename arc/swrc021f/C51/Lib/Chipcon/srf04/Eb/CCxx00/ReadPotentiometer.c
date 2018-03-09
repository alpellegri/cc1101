/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements functionality for reading the potmeter                *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     ReadPotentiometer.c                                                        *
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
//  UINT8 ebReadPotentiometer(void)
//
//  DESCRIPTION:
//      This function reads the potmeter located at the SmartRF04EB using the ADC. The function only 
//      reads the 8 MSBs from the ADC. The ebAdcinit(ADC_POT) function must be called before this 
//      function can be called.
//
//  RETURN VALUE:
//      UINT8
//          8 bit sample from ADC
//-------------------------------------------------------------------------------------------------------
UINT8 ebReadPotentiometer(void) {

    // Sample the ADC
    ADC_SAMPLE();

    return ADC0H;
}// ebReadPotentiometer




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: ReadPotentiometer.c,v $
 * Revision 1.2  2005/02/07 14:43:55  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
