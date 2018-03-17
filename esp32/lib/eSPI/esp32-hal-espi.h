// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAIN_ESP32_HAL_ESPI_H_
#define MAIN_ESP32_HAL_ESPI_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define ESPI_HAS_TRANSACTION

#define FSPI                                                                   \
  1 // SPI bus attached to the flash (can use the same data lines but different
    // SS)
#define HSPI                                                                   \
  2 // SPI bus normally mapped to pins 12 - 15, but can be matrixed to any pins
#define VSPI                                                                   \
  3 // SPI bus normally attached to pins 5, 18, 19 and 23, but can be matrixed
    // to any pins

// This defines are not representing the real Divider of the ESP32
// the Defines match to an AVR Arduino on 16MHz for better compatibility
#define ESPI_CLOCK_DIV2 0x00101001   // 8 MHz
#define ESPI_CLOCK_DIV4 0x00241001   // 4 MHz
#define ESPI_CLOCK_DIV8 0x004c1001   // 2 MHz
#define ESPI_CLOCK_DIV16 0x009c1001  // 1 MHz
#define ESPI_CLOCK_DIV32 0x013c1001  // 500 KHz
#define ESPI_CLOCK_DIV64 0x027c1001  // 250 KHz
#define ESPI_CLOCK_DIV128 0x04fc1001 // 125 KHz

#define ESPI_MODE0 0
#define ESPI_MODE1 1
#define ESPI_MODE2 2
#define ESPI_MODE3 3

#define ESPI_CS0 0
#define ESPI_CS1 1
#define ESPI_CS2 2
#define ESPI_CS_MASK_ALL 0x7

#define ESPI_LSBFIRST 0
#define ESPI_MSBFIRST 1

struct spi_struct_t;
typedef struct spi_struct_t espi_t;

espi_t *espiStartBus(uint8_t spi_num, uint32_t freq, uint8_t dataMode,
                     uint8_t bitOrder);
void espiStopBus(espi_t *spi);

// Attach/Detach Signal Pins
void espiAttachSCK(espi_t *spi, int8_t sck);
void espiAttachMISO(espi_t *spi, int8_t miso);
void espiAttachMOSI(espi_t *spi, int8_t mosi);
void espiDetachSCK(espi_t *spi, int8_t sck);
void espiDetachMISO(espi_t *spi, int8_t miso);
void espiDetachMOSI(espi_t *spi, int8_t mosi);

// Attach/Detach SS pin to ESPI_CSx signal
void espiAttachSS(espi_t *spi, uint8_t cs_num, int8_t ss);
void espiDetachSS(espi_t *spi, int8_t ss);

// Enable/Disable ESPI_CSx pins
void espiEnableSSPins(espi_t *spi, uint8_t cs_mask);
void espiDisableSSPins(espi_t *spi, uint8_t cs_mask);

// Enable/Disable hardware control of ESPI_CSx pins
void espiSSEnable(espi_t *spi);
void espiSSDisable(espi_t *spi);

// Activate enabled ESPI_CSx pins
void espiSSSet(espi_t *spi);
// Deactivate enabled ESPI_CSx pins
void espiSSClear(espi_t *spi);

void espiWaitReady(espi_t *spi);

uint32_t espiGetClockDiv(espi_t *spi);
uint8_t espiGetDataMode(espi_t *spi);
uint8_t espiGetBitOrder(espi_t *spi);

/*
 * Non transaction based lock methods (each locks and unlocks when called)
 * */
void espiSetClockDiv(espi_t *spi, uint32_t clockDiv);
void espiSetDataMode(espi_t *spi, uint8_t dataMode);
void espiSetBitOrder(espi_t *spi, uint8_t bitOrder);

void espiWrite(espi_t *spi, uint32_t *data, uint8_t len);
void espiWriteByte(espi_t *spi, uint8_t data);
void espiWriteWord(espi_t *spi, uint16_t data);
void espiWriteLong(espi_t *spi, uint32_t data);

uint8_t espiRead(espi_t *spi);

void espiTransfer(espi_t *spi, uint32_t *out, uint8_t len);
uint8_t espiTransferByte(espi_t *spi, uint8_t data);
uint16_t espiTransferWord(espi_t *spi, uint16_t data);
uint32_t espiTransferLong(espi_t *spi, uint32_t data);
void espiTransferBytes(espi_t *spi, uint8_t *data, uint8_t *out, uint32_t size);
void espiTransferBits(espi_t *spi, uint32_t data, uint32_t *out, uint8_t bits);

/*
 * New (EXPERIMENTAL) Transaction lock based API (lock once until
 * endTransaction)
 * */
void espiTransaction(espi_t *spi, uint32_t clockDiv, uint8_t dataMode,
                     uint8_t bitOrder);
void espiSimpleTransaction(espi_t *spi);
void espiEndTransaction(espi_t *spi);

void espiWriteNL(espi_t *spi, const void *data, uint32_t len);
void espiWriteByteNL(espi_t *spi, uint8_t data);
void espiWriteShortNL(espi_t *spi, uint16_t data);
void espiWriteLongNL(espi_t *spi, uint32_t data);
void espiWritePixelsNL(espi_t *spi, const void *data, uint32_t len);

#define espiTransferNL(spi, data, len) spiTransferBytesNL(spi, data, data, len)
uint8_t espiTransferByteNL(espi_t *spi, uint8_t data);
uint16_t espiTransferShortNL(espi_t *spi, uint16_t data);
uint32_t espiTransferLongNL(espi_t *spi, uint32_t data);
void espiTransferBytesNL(espi_t *spi, const void *data_in, uint8_t *data_out,
                         uint32_t len);
void espiTransferBitsNL(espi_t *spi, uint32_t data_in, uint32_t *data_out,
                        uint8_t bits);

/*
 * Helper functions to translate frequency to clock divider and back
 * */
uint32_t espiFrequencyToClockDiv(uint32_t freq);
uint32_t espiClockDivToFrequency(uint32_t freq);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_ESP32_HAL_ESPI_H_ */
