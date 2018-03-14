#include <Arduino.h>

#include <stdio.h>
#include <string.h>

#include "cc1101.h"

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

CC1101 cc1101;

void setup() {
  uint8_t data;
  Serial.begin(115200);

  cc1101.init();

  // setup the blinker output
  pinMode(LEDOUTPUT, OUTPUT);
  digitalWrite(LEDOUTPUT, LOW);

  Serial.println("");
  Serial.print("CC1101_VERSION ");
  data = cc1101.readStatus(CC1101_VERSION);
  Serial.println(data);
  Serial.print("CC1101_PKTCTRL0 ");
  data = cc1101.readReg(CC1101_PKTCTRL0);
  Serial.println(data);
  Serial.print("CC1101_MDMCFG2 ");
  data = cc1101.readReg(CC1101_MDMCFG2);
  Serial.println(data);
}

uint32_t schedule_time;
uint32_t fame_cnt;
uint32_t sts = 0;

void loop() {
  uint8_t data;
  uint8_t Buffer[128];
  uint8_t Buffer2[128];
  uint16_t BufferLen;
  uint16_t i;

  if (sts == 0) {
    cc1101.receiveNb(Buffer, &BufferLen);
    sts = 1;
  } else if (sts == 1) {
    uint16_t len = cc1101.receiveNbReady();
    if (len != 0) {
      Serial.printf("Packet length: %d\n", len);
      for (i = 0; i < len; i++) {
        Serial.printf("%02X ", (uint8_t)Buffer[i]);
      }
      Serial.printf("\n");

      manch_dec(Buffer, Buffer2, len);
      Serial.printf("Packet length: %d\n", len);
      for (i = 0; i < len; i++) {
        Serial.printf("%02X ", (uint8_t)Buffer2[i]);
      }
      Serial.printf("\n");
      cc1101.receiveNb(Buffer, &BufferLen);
    }
  }

  uint32_t current_time = millis();
  if ((current_time - schedule_time) > 1000) {
    schedule_time = current_time;
    digitalWrite(LEDOUTPUT, LOW);

#if 0
    // create a random packet with PKTLEN + 2 byte packet counter + n x random
    // bytes
    createPacket(Buffer);

    // write packet to tx fifo
    cc1101.send(Buffer, PKTLEN + 1);
#else
#if 1
    // cc1101.receiveNb(Buffer, &BufferLen);
#else
    fame_cnt++;
    data = cc1101.readStatus(CC1101_MARCSTATE);
    Serial.println(data);
    data = cc1101.getStatus();
    Serial.println(data);
    memset(Buffer, 0x00, sizeof(Buffer));
    cc1101.receive(Buffer, &BufferLen);
    Serial.printf("%d Packet length: %d\n", fame_cnt, BufferLen);
    for (i = 0; i < BufferLen; i++) {
      Serial.printf("%02X ", (uint8_t)Buffer[i]);
    }
    Serial.printf("\n", BufferLen);
#endif
#endif

    digitalWrite(LEDOUTPUT, HIGH);
  }
}
