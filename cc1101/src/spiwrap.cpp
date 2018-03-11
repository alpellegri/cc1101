/******************************************************************************
  Filename:

  Description:

  Notes:

******************************************************************************/

#include <Arduino.h>
#include <SPI.h>

#include "common.h"
#include "spiwrap.h"

BYTE SPI_Rx(void) { return (uint8_t)(SPI1W0 & 0xff); }

void CTRL_WAIT_SYNC(BYTE pin) {
  while(digitalRead(pin) == false) {
    delay(1);
  }
}

void CTRL_WAIT_EOT(BYTE pin) {
  while(digitalRead(pin) == true) {
    delay(1);
  }
}

void ICACHE_RAM_ATTR SPI_BEGIN(void) { digitalWrite(SS, LOW); }
void ICACHE_RAM_ATTR SPI_WAIT(void) {}
void ICACHE_RAM_ATTR SPI_WAIT_MISO(void) {}
void ICACHE_RAM_ATTR SPI_WAIT_DONE(void) {}
void ICACHE_RAM_ATTR SPI_TX(BYTE x) { SPI.transfer(x); }
BYTE ICACHE_RAM_ATTR SPI_RX(void) { return SPI_Rx(); }
void ICACHE_RAM_ATTR SPI_END(void) { digitalWrite(SS, HIGH); }

void (*cb1)(void);

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
uint8_t cnt;
void ICACHE_RAM_ATTR Interrupt1(void) {
  Serial.println(cnt++);
  cb1();
}

void SetCb1(BYTE pin, void (*cb)(void)) {
  cb1 = cb;
  attachInterrupt(pin, Interrupt1, FALLING);
}

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
