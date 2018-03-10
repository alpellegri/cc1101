/******************************************************************************
  Filename:

  Description:

  Notes:

******************************************************************************/

/*****************************************************************************
 * INCLUDES
 */

#include <Arduino.h>
#include <SPI.h>

#include "common.h"
#include "spiwrap.h"

BYTE SPI_Rx(void) { return (uint8_t)(SPI1W0 & 0xff); }

void SPI_BEGIN(void) { digitalWrite(SS, LOW); }
void SPI_WAIT(void) {}
void SPI_WAIT_MISO(void) {}
void SPI_WAIT_DONE(void) {}
void SPI_TX(BYTE x) { SPI.transfer(x); }
BYTE SPI_RX(void) { return SPI_Rx(); }
void SPI_END(void) { digitalWrite(SS, HIGH); }

/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *
 * @param       input, output parameters
 *
 * @return      describe return value, if any
 */
void SPI_INIT(void) {
  // SPI.setHwCs(SS);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  SPI.setFrequency(1000000);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}
