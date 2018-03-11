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
}

uint32_t schedule_time;
uint32_t fame_cnt;

void loop() {
  uint8_t data;
  uint8 Buffer[PKTLEN + 1] = {0};
  uint8 BufferLen;
  uint16_t i;

  uint32_t current_time = millis();
  if ((current_time - schedule_time) > 1000) {
    schedule_time = current_time;
    digitalWrite(LEDOUTPUT, LOW);

#if 0
    // create a random packet with PKTLEN + 2 byte packet counter + n x random
    // bytes
    createPacket(Buffer);

    // write packet to tx fifo
    cc1101.send(Buffer, sizeof(Buffer));
#else
#if 0
    // cc1101.strobe(CC1101_SRX);
#else
    fame_cnt++;
    data = cc1101.readStatus(CC1101_MARCSTATE);
    Serial.println(data);
    data = cc1101.getStatus();
    Serial.println(data);
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
