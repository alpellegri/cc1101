#include <Arduino.h>

#include <stdio.h>
#include <string.h>

#include "cc1101_conf.h"
#include "cc1101_spi.h"

#define LEDOUTPUT 16
#define PKTLEN 30

static uint8_t packetSemaphore;
static uint32_t packetCounter;

static void createPacket(uint8_t txBuffer[]) {
  txBuffer[0] = PKTLEN;                        // Length byte
  txBuffer[1] = (uint8_t)(packetCounter >> 8); // MSB of packetCounter
  txBuffer[2] = (uint8_t)packetCounter;        // LSB of packetCounter

  // fill rest of buffer with random bytes
  for (uint8 i = 3; i < (PKTLEN + 1); i++) {
    // txBuffer[i] = (uint8_t)rand();
    txBuffer[i] = (uint8_t)i;
  }
}

void setup() {
  uint8_t data;
  Serial.begin(115200);

  cc1101Init();

  // setup the blinker output
  pinMode(LEDOUTPUT, OUTPUT);
  digitalWrite(LEDOUTPUT, LOW);

  Serial.println();
  Serial.print("CC1101_PARTNUM ");
  cc1101SpiReadReg(CC1101_PARTNUM, &data, 1);
  Serial.println(data);
  Serial.print("CC1101_VERSION ");
  cc1101SpiReadReg(CC1101_VERSION, &data, 1);
  Serial.println(data);

  registerConfig();
}

uint32_t schedule_time;

void loop() {
  uint8_t data;
  uint8 txBuffer[PKTLEN + 1] = {0};

  uint32_t current_time = millis();
  if ((current_time - schedule_time) > 1000) {
    schedule_time = current_time;
    digitalWrite(LEDOUTPUT, LOW);

    Serial.println();
    Serial.print("CC1101_PARTNUM ");
    cc1101SpiReadReg(CC1101_PARTNUM, &data, 1);
    Serial.println(data);
    Serial.print("CC1101_VERSION ");
    cc1101SpiReadReg(CC1101_VERSION, &data, 1);
    Serial.println(data);

#if 1
    // create a random packet with PKTLEN + 2 byte packet counter + n x random
    // bytes
    createPacket(txBuffer);

    // write packet to tx fifo
    cc1101SpiWriteTxFifo(txBuffer, sizeof(txBuffer));

    // strobe TX to send packet
    trxSpiCmdStrobe(CC1101_STX);
#endif
    digitalWrite(LEDOUTPUT, HIGH);
  }
}
