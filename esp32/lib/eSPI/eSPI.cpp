/*
 SPI.cpp - SPI library for esp8266
 Copyright (c) 2015 Hristo Gochkov. All rights reserved.
 This file is part of the esp8266 core for Arduino environment.

 This library is free software; you can redistribute it and/or
 modify it under the terms of the GNU Lesser General Public
 License as published by the Free Software Foundation; either
 version 2.1 of the License, or (at your option) any later version.
 This library is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 Lesser General Public License for more details.
 You should have received a copy of the GNU Lesser General Public
 License along with this library; if not, write to the Free Software
 Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "eSPI.h"

eSPIClass::eSPIClass(uint8_t spi_bus)
    : _spi_num(spi_bus), _spi(NULL), _use_hw_ss(false), _sck(-1), _miso(-1),
      _mosi(-1), _ss(-1), _div(0), _freq(1000000), _inTransaction(false) {}

void eSPIClass::begin(int8_t sck, int8_t miso, int8_t mosi, int8_t ss) {
  if (_spi) {
    return;
  }

  if (!_div) {
    _div = espiFrequencyToClockDiv(_freq);
  }

  _spi = espiStartBus(_spi_num, _div, ESPI_MODE0, ESPI_MSBFIRST);
  if (!_spi) {
    return;
  }

  if (sck == -1 && miso == -1 && mosi == -1 && ss == -1) {
    _sck = (_spi_num == VSPI) ? SCK : 14;
    _miso = (_spi_num == VSPI) ? MISO : 12;
    _mosi = (_spi_num == VSPI) ? MOSI : 13;
    _ss = (_spi_num == VSPI) ? SS : 15;
  } else {
    _sck = sck;
    _miso = miso;
    _mosi = mosi;
    _ss = ss;
  }

  espiAttachSCK(_spi, _sck);
  espiAttachMISO(_spi, _miso);
  espiAttachMOSI(_spi, _mosi);
}

void eSPIClass::end() {
  if (!_spi) {
    return;
  }
  espiDetachSCK(_spi, _sck);
  espiDetachMISO(_spi, _miso);
  espiDetachMOSI(_spi, _mosi);
  setHwCs(false);
  espiStopBus(_spi);
  _spi = NULL;
}

void eSPIClass::setHwCs(bool use) {
  if (use && !_use_hw_ss) {
    espiAttachSS(_spi, 0, _ss);
    espiSSEnable(_spi);
  } else if (_use_hw_ss) {
    espiSSDisable(_spi);
    espiDetachSS(_spi, _ss);
  }
  _use_hw_ss = use;
}

void eSPIClass::setFrequency(uint32_t freq) {
  // check if last freq changed
  uint32_t cdiv = espiGetClockDiv(_spi);
  if (_freq != freq || _div != cdiv) {
    _freq = freq;
    _div = espiFrequencyToClockDiv(_freq);
    espiSetClockDiv(_spi, _div);
  }
}

void eSPIClass::setClockDivider(uint32_t clockDiv) {
  _div = clockDiv;
  espiSetClockDiv(_spi, _div);
}

void eSPIClass::setDataMode(uint8_t dataMode) {
  espiSetDataMode(_spi, dataMode);
}

void eSPIClass::setBitOrder(uint8_t bitOrder) {
  espiSetBitOrder(_spi, bitOrder);
}

void eSPIClass::beginTransaction(eSPISettings settings) {
  // check if last freq changed
  uint32_t cdiv = espiGetClockDiv(_spi);
  if (_freq != settings._clock || _div != cdiv) {
    _freq = settings._clock;
    _div = espiFrequencyToClockDiv(_freq);
  }
  espiTransaction(_spi, _div, settings._dataMode, settings._bitOrder);
  _inTransaction = true;
}

void eSPIClass::endTransaction() {
  if (_inTransaction) {
    _inTransaction = false;
    espiEndTransaction(_spi);
  }
}

void eSPIClass::write(uint8_t data) {
  if (_inTransaction) {
    return espiWriteByteNL(_spi, data);
  }
  espiWriteByte(_spi, data);
}

uint8_t eSPIClass::transfer(uint8_t data) {
  if (_inTransaction) {
    return espiTransferByteNL(_spi, data);
  }
  return espiTransferByte(_spi, data);
}

uint8_t eSPIClass::read(void) { return espiRead(_spi); }

void eSPIClass::write16(uint16_t data) {
  if (_inTransaction) {
    return espiWriteShortNL(_spi, data);
  }
  espiWriteWord(_spi, data);
}

uint16_t eSPIClass::transfer16(uint16_t data) {
  if (_inTransaction) {
    return espiTransferShortNL(_spi, data);
  }
  return espiTransferWord(_spi, data);
}

void eSPIClass::write32(uint32_t data) {
  if (_inTransaction) {
    return espiWriteLongNL(_spi, data);
  }
  espiWriteLong(_spi, data);
}

uint32_t eSPIClass::transfer32(uint32_t data) {
  if (_inTransaction) {
    return espiTransferLongNL(_spi, data);
  }
  return espiTransferLong(_spi, data);
}

void eSPIClass::transferBits(uint32_t data, uint32_t *out, uint8_t bits) {
  if (_inTransaction) {
    return espiTransferBitsNL(_spi, data, out, bits);
  }
  espiTransferBits(_spi, data, out, bits);
}

/**
 * @param data uint8_t *
 * @param size uint32_t
 */
void eSPIClass::writeBytes(uint8_t *data, uint32_t size) {
  if (_inTransaction) {
    return espiWriteNL(_spi, data, size);
  }
  espiSimpleTransaction(_spi);
  espiWriteNL(_spi, data, size);
  espiEndTransaction(_spi);
}

/**
 * @param data void *
 * @param size uint32_t
 */
void eSPIClass::writePixels(const void *data, uint32_t size) {
  if (_inTransaction) {
    return espiWritePixelsNL(_spi, data, size);
  }
  espiSimpleTransaction(_spi);
  espiWritePixelsNL(_spi, data, size);
  espiEndTransaction(_spi);
}

/**
 * @param data uint8_t * data buffer. can be NULL for Read Only operation
 * @param out  uint8_t * output buffer. can be NULL for Write Only operation
 * @param size uint32_t
 */
void eSPIClass::transferBytes(uint8_t *data, uint8_t *out, uint32_t size) {
  if (_inTransaction) {
    return espiTransferBytesNL(_spi, data, out, size);
  }
  espiTransferBytes(_spi, data, out, size);
}

/**
 * @param data uint8_t *
 * @param size uint8_t  max for size is 64Byte
 * @param repeat uint32_t
 */
void eSPIClass::writePattern(uint8_t *data, uint8_t size, uint32_t repeat) {
  if (size > 64) {
    return; // max Hardware FIFO
  }

  uint32_t byte = (size * repeat);
  uint8_t r = (64 / size);
  const uint8_t max_bytes_FIFO = r * size; // Max number of whole patterns (in
                                           // bytes) that can fit into the
                                           // hardware FIFO

  while (byte) {
    if (byte > max_bytes_FIFO) {
      writePattern_(data, size, r);
      byte -= max_bytes_FIFO;
    } else {
      writePattern_(data, size, (byte / size));
      byte = 0;
    }
  }
}

void eSPIClass::writePattern_(uint8_t *data, uint8_t size, uint8_t repeat) {
  uint8_t bytes = (size * repeat);
  uint8_t buffer[64];
  uint8_t *bufferPtr = &buffer[0];
  uint8_t *dataPtr;
  uint8_t dataSize = bytes;
  for (uint8_t i = 0; i < repeat; i++) {
    dataSize = size;
    dataPtr = data;
    while (dataSize--) {
      *bufferPtr = *dataPtr;
      dataPtr++;
      bufferPtr++;
    }
  }

  writeBytes(&buffer[0], bytes);
}

eSPIClass eSPI(VSPI);
