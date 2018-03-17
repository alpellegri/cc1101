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

#include "esp32-hal-espi.h"
#include "esp32-hal.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "rom/ets_sys.h"
#include "rom/gpio.h"
#include "soc/dport_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/io_mux_reg.h"
#include "soc/spi_reg.h"
#include "soc/spi_struct.h"

#define SPI_CLK_IDX(p)                                                         \
  ((p == 0) ? SPICLK_OUT_IDX                                                   \
            : ((p == 1) ? SPICLK_OUT_IDX                                       \
                        : ((p == 2) ? HSPICLK_OUT_IDX                          \
                                    : ((p == 3) ? VSPICLK_OUT_IDX : 0))))
#define SPI_MISO_IDX(p)                                                        \
  ((p == 0) ? SPIQ_OUT_IDX                                                     \
            : ((p == 1) ? SPIQ_OUT_IDX                                         \
                        : ((p == 2) ? HSPIQ_OUT_IDX                            \
                                    : ((p == 3) ? VSPIQ_OUT_IDX : 0))))
#define SPI_MOSI_IDX(p)                                                        \
  ((p == 0) ? SPID_IN_IDX                                                      \
            : ((p == 1) ? SPID_IN_IDX                                          \
                        : ((p == 2) ? HSPID_IN_IDX                             \
                                    : ((p == 3) ? VSPID_IN_IDX : 0))))

#define SPI_SPI_SS_IDX(n)                                                      \
  ((n == 0) ? SPICS0_OUT_IDX                                                   \
            : ((n == 1) ? SPICS1_OUT_IDX                                       \
                        : ((n == 2) ? SPICS2_OUT_IDX : SPICS0_OUT_IDX)))
#define SPI_HSPI_SS_IDX(n)                                                     \
  ((n == 0) ? HSPICS0_OUT_IDX                                                  \
            : ((n == 1) ? HSPICS1_OUT_IDX                                      \
                        : ((n == 2) ? HSPICS2_OUT_IDX : HSPICS0_OUT_IDX)))
#define SPI_VSPI_SS_IDX(n)                                                     \
  ((n == 0) ? VSPICS0_OUT_IDX                                                  \
            : ((n == 1) ? VSPICS1_OUT_IDX                                      \
                        : ((n == 2) ? VSPICS2_OUT_IDX : VSPICS0_OUT_IDX)))
#define SPI_SS_IDX(p, n)                                                       \
  ((p == 0) ? SPI_SPI_SS_IDX(n)                                                \
            : ((p == 1) ? SPI_SPI_SS_IDX(n)                                    \
                        : ((p == 2) ? SPI_HSPI_SS_IDX(n)                       \
                                    : ((p == 3) ? SPI_VSPI_SS_IDX(n) : 0))))

#define SPI_INUM(u) (2)
#define SPI_INTR_SOURCE(u)                                                     \
  ((u == 0) ? ETS_SPI0_INTR_SOURCE                                             \
            : ((u == 1) ? ETS_SPI1_INTR_SOURCE                                 \
                        : ((u == 2) ? ETS_SPI2_INTR_SOURCE                     \
                                    : ((p == 3) ? ETS_SPI3_INTR_SOURCE : 0))))

struct spi_struct_t {
  spi_dev_t *dev;
#if !CONFIG_DISABLE_HAL_LOCKS
  xSemaphoreHandle lock;
#endif
  uint8_t num;
};

#if CONFIG_DISABLE_HAL_LOCKS
#define SPI_MUTEX_LOCK()
#define SPI_MUTEX_UNLOCK()

static espi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), 3}};
#else
#define SPI_MUTEX_LOCK()                                                       \
  do {                                                                         \
  } while (xSemaphoreTake(spi->lock, portMAX_DELAY) != pdPASS)
#define SPI_MUTEX_UNLOCK() xSemaphoreGive(spi->lock)

static espi_t _spi_bus_array[4] = {
    {(volatile spi_dev_t *)(DR_REG_SPI0_BASE), NULL, 0},
    {(volatile spi_dev_t *)(DR_REG_SPI1_BASE), NULL, 1},
    {(volatile spi_dev_t *)(DR_REG_SPI2_BASE), NULL, 2},
    {(volatile spi_dev_t *)(DR_REG_SPI3_BASE), NULL, 3}};
#endif

void espiAttachSCK(espi_t *spi, int8_t sck) {
  if (!spi) {
    return;
  }
  if (sck < 0) {
    if (spi->num == HSPI) {
      sck = 14;
    } else if (spi->num == VSPI) {
      sck = 18;
    } else {
      sck = 6;
    }
  }
  pinMode(sck, OUTPUT);
  pinMatrixOutAttach(sck, SPI_CLK_IDX(spi->num), false, false);
}

void espiAttachMISO(espi_t *spi, int8_t miso) {
  if (!spi) {
    return;
  }
  if (miso < 0) {
    if (spi->num == HSPI) {
      miso = 12;
    } else if (spi->num == VSPI) {
      miso = 19;
    } else {
      miso = 7;
    }
  }
  SPI_MUTEX_LOCK();
  pinMode(miso, INPUT);
  pinMatrixInAttach(miso, SPI_MISO_IDX(spi->num), false);
  SPI_MUTEX_UNLOCK();
}

void espiAttachMOSI(espi_t *spi, int8_t mosi) {
  if (!spi) {
    return;
  }
  if (mosi < 0) {
    if (spi->num == HSPI) {
      mosi = 13;
    } else if (spi->num == VSPI) {
      mosi = 23;
    } else {
      mosi = 8;
    }
  }
  pinMode(mosi, OUTPUT);
  pinMatrixOutAttach(mosi, SPI_MOSI_IDX(spi->num), false, false);
}

void espiDetachSCK(espi_t *spi, int8_t sck) {
  if (!spi) {
    return;
  }
  if (sck < 0) {
    if (spi->num == HSPI) {
      sck = 14;
    } else if (spi->num == VSPI) {
      sck = 18;
    } else {
      sck = 6;
    }
  }
  pinMatrixOutDetach(sck, false, false);
  pinMode(sck, INPUT);
}

void espiDetachMISO(espi_t *spi, int8_t miso) {
  if (!spi) {
    return;
  }
  if (miso < 0) {
    if (spi->num == HSPI) {
      miso = 12;
    } else if (spi->num == VSPI) {
      miso = 19;
    } else {
      miso = 7;
    }
  }
  pinMatrixInDetach(SPI_MISO_IDX(spi->num), false, false);
  pinMode(miso, INPUT);
}

void espiDetachMOSI(espi_t *spi, int8_t mosi) {
  if (!spi) {
    return;
  }
  if (mosi < 0) {
    if (spi->num == HSPI) {
      mosi = 13;
    } else if (spi->num == VSPI) {
      mosi = 23;
    } else {
      mosi = 8;
    }
  }
  pinMatrixOutDetach(mosi, false, false);
  pinMode(mosi, INPUT);
}

void espiAttachSS(espi_t *spi, uint8_t cs_num, int8_t ss) {
  if (!spi) {
    return;
  }
  if (cs_num > 2) {
    return;
  }
  if (ss < 0) {
    cs_num = 0;
    if (spi->num == HSPI) {
      ss = 15;
    } else if (spi->num == VSPI) {
      ss = 5;
    } else {
      ss = 11;
    }
  }
  pinMode(ss, OUTPUT);
  pinMatrixOutAttach(ss, SPI_SS_IDX(spi->num, cs_num), false, false);
  spiEnableSSPins(spi, (1 << cs_num));
}

void espiDetachSS(espi_t *spi, int8_t ss) {
  if (!spi) {
    return;
  }
  if (ss < 0) {
    if (spi->num == HSPI) {
      ss = 15;
    } else if (spi->num == VSPI) {
      ss = 5;
    } else {
      ss = 11;
    }
  }
  pinMatrixOutDetach(ss, false, false);
  pinMode(ss, INPUT);
}

void espiEnableSSPins(espi_t *spi, uint8_t cs_mask) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->pin.val &= ~(cs_mask & SPI_CS_MASK_ALL);
  SPI_MUTEX_UNLOCK();
}

void espiDisableSSPins(espi_t *spi, uint8_t cs_mask) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->pin.val |= (cs_mask & SPI_CS_MASK_ALL);
  SPI_MUTEX_UNLOCK();
}

void espiSSEnable(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->user.cs_setup = 1;
  spi->dev->user.cs_hold = 1;
  SPI_MUTEX_UNLOCK();
}

void espiSSDisable(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->user.cs_setup = 0;
  spi->dev->user.cs_hold = 0;
  SPI_MUTEX_UNLOCK();
}

void espiSSSet(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->pin.cs_keep_active = 1;
  SPI_MUTEX_UNLOCK();
}

void espiSSClear(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->pin.cs_keep_active = 0;
  SPI_MUTEX_UNLOCK();
}

uint32_t espiGetClockDiv(espi_t *spi) {
  if (!spi) {
    return 0;
  }
  return spi->dev->clock.val;
}

void espiSetClockDiv(espi_t *spi, uint32_t clockDiv) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->clock.val = clockDiv;
  SPI_MUTEX_UNLOCK();
}

uint8_t espiGetDataMode(espi_t *spi) {
  if (!spi) {
    return 0;
  }
  bool idleEdge = spi->dev->pin.ck_idle_edge;
  bool outEdge = spi->dev->user.ck_out_edge;
  if (idleEdge) {
    if (outEdge) {
      return SPI_MODE2;
    }
    return SPI_MODE3;
  }
  if (outEdge) {
    return SPI_MODE1;
  }
  return SPI_MODE0;
}

void espiSetDataMode(espi_t *spi, uint8_t dataMode) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  switch (dataMode) {
  case ESPI_MODE1:
    spi->dev->pin.ck_idle_edge = 0;
    spi->dev->user.ck_out_edge = 1;
    break;
  case ESPI_MODE2:
    spi->dev->pin.ck_idle_edge = 1;
    spi->dev->user.ck_out_edge = 1;
    break;
  case ESPI_MODE3:
    spi->dev->pin.ck_idle_edge = 1;
    spi->dev->user.ck_out_edge = 0;
    break;
  case ESPI_MODE0:
  default:
    spi->dev->pin.ck_idle_edge = 0;
    spi->dev->user.ck_out_edge = 0;
    break;
  }
  SPI_MUTEX_UNLOCK();
}

uint8_t espiGetBitOrder(espi_t *spi) {
  if (!spi) {
    return 0;
  }
  return (spi->dev->ctrl.wr_bit_order | spi->dev->ctrl.rd_bit_order) == 0;
}

void espiSetBitOrder(espi_t *spi, uint8_t bitOrder) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  if (ESPI_MSBFIRST == bitOrder) {
    spi->dev->ctrl.wr_bit_order = 0;
    spi->dev->ctrl.rd_bit_order = 0;
  } else if (ESPI_LSBFIRST == bitOrder) {
    spi->dev->ctrl.wr_bit_order = 1;
    spi->dev->ctrl.rd_bit_order = 1;
  }
  SPI_MUTEX_UNLOCK();
}

void espiStopBus(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->slave.trans_done = 0;
  spi->dev->slave.slave_mode = 0;
  spi->dev->pin.val = 0;
  spi->dev->user.val = 0;
  spi->dev->user1.val = 0;
  spi->dev->ctrl.val = 0;
  spi->dev->ctrl1.val = 0;
  spi->dev->ctrl2.val = 0;
  spi->dev->clock.val = 0;
  SPI_MUTEX_UNLOCK();
}

espi_t *espiStartBus(uint8_t spi_num, uint32_t clockDiv, uint8_t dataMode,
                    uint8_t bitOrder) {
  if (spi_num > 3) {
    return NULL;
  }

  espi_t *spi = &_spi_bus_array[spi_num];

#if !CONFIG_DISABLE_HAL_LOCKS
  if (spi->lock == NULL) {
    spi->lock = xSemaphoreCreateMutex();
    if (spi->lock == NULL) {
      return NULL;
    }
  }
#endif

  if (spi_num == HSPI) {
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST);
  } else if (spi_num == VSPI) {
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_2);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST_2);
  } else {
    DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_SPI_CLK_EN_1);
    DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_SPI_RST_1);
  }

  espiStopBus(spi);
  espiSetDataMode(spi, dataMode);
  espiSetBitOrder(spi, bitOrder);
  espiSetClockDiv(spi, clockDiv);

  SPI_MUTEX_LOCK();
  spi->dev->user.usr_mosi = 1;
  spi->dev->user.usr_miso = 1;
  spi->dev->user.doutdin = 1;

  int i;
  for (i = 0; i < 16; i++) {
    spi->dev->data_buf[i] = 0x00000000;
  }
  SPI_MUTEX_UNLOCK();

  return spi;
}

void espiWaitReady(espi_t *spi) {
  if (!spi) {
    return;
  }
  while (spi->dev->cmd.usr)
    ;
}

void espiWrite(espi_t *spi, uint32_t *data, uint8_t len) {
  if (!spi) {
    return;
  }
  int i;
  if (len > 16) {
    len = 16;
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = (len * 32) - 1;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  for (i = 0; i < len; i++) {
    spi->dev->data_buf[i] = data[i];
  }
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  SPI_MUTEX_UNLOCK();
}

void espiTransfer(espi_t *spi, uint32_t *data, uint8_t len) {
  if (!spi) {
    return;
  }
  int i;
  if (len > 16) {
    len = 16;
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = (len * 32) - 1;
  spi->dev->miso_dlen.usr_miso_dbitlen = (len * 32) - 1;
  for (i = 0; i < len; i++) {
    spi->dev->data_buf[i] = data[i];
  }
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  for (i = 0; i < len; i++) {
    data[i] = spi->dev->data_buf[i];
  }
  SPI_MUTEX_UNLOCK();
}

uint8_t espiRead(espi_t *spi) {
  return spi->dev->data_buf[0] & 0xFF;
}

void espiWriteByte(espi_t *spi, uint8_t data) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  SPI_MUTEX_UNLOCK();
}

uint8_t espiTransferByte(espi_t *spi, uint8_t data) {
  if (!spi) {
    return 0;
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
  spi->dev->miso_dlen.usr_miso_dbitlen = 7;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0] & 0xFF;
  SPI_MUTEX_UNLOCK();
  return data;
}

uint32_t __espiTranslate24(uint32_t data) {
  union {
    uint32_t l;
    uint8_t b[4];
  } out;
  out.l = data;
  return out.b[2] | (out.b[1] << 8) | (out.b[0] << 16);
}

uint32_t __espiTranslate32(uint32_t data) {
  union {
    uint32_t l;
    uint8_t b[4];
  } out;
  out.l = data;
  return out.b[3] | (out.b[2] << 8) | (out.b[1] << 16) | (out.b[0] << 24);
}

void espiWriteWord(espi_t *spi, uint16_t data) {
  if (!spi) {
    return;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    data = (data >> 8) | (data << 8);
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  SPI_MUTEX_UNLOCK();
}

uint16_t espiTransferWord(espi_t *spi, uint16_t data) {
  if (!spi) {
    return 0;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    data = (data >> 8) | (data << 8);
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
  spi->dev->miso_dlen.usr_miso_dbitlen = 15;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0];
  SPI_MUTEX_UNLOCK();
  if (!spi->dev->ctrl.rd_bit_order) {
    data = (data >> 8) | (data << 8);
  }
  return data;
}

void espiWriteLong(espi_t *spi, uint32_t data) {
  if (!spi) {
    return;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    data = __espiTranslate32(data);
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 31;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  SPI_MUTEX_UNLOCK();
}

uint32_t espiTransferLong(espi_t *spi, uint32_t data) {
  if (!spi) {
    return 0;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    data = __espiTranslate32(data);
  }
  SPI_MUTEX_LOCK();
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 31;
  spi->dev->miso_dlen.usr_miso_dbitlen = 31;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0];
  SPI_MUTEX_UNLOCK();
  if (!spi->dev->ctrl.rd_bit_order) {
    data = __espiTranslate32(data);
  }
  return data;
}

void __espiTransferBytes(espi_t *spi, uint8_t *data, uint8_t *out,
                        uint32_t bytes) {
  if (!spi) {
    return;
  }
  int i;

  if (bytes > 64) {
    bytes = 64;
  }

  uint32_t words = (bytes + 3) / 4; // 16 max

  uint32_t wordsBuf[16] = {
      0,
  };
  uint8_t *bytesBuf = (uint8_t *)wordsBuf;

  if (data) {
    memcpy(bytesBuf, data, bytes); // copy data to buffer
  } else {
    memset(bytesBuf, 0xFF, bytes);
  }

  spi->dev->mosi_dlen.usr_mosi_dbitlen = ((bytes * 8) - 1);
  spi->dev->miso_dlen.usr_miso_dbitlen = ((bytes * 8) - 1);

  for (i = 0; i < words; i++) {
    spi->dev->data_buf[i] = wordsBuf[i]; // copy buffer to spi fifo
  }

  spi->dev->cmd.usr = 1;

  while (spi->dev->cmd.usr)
    ;

  if (out) {
    for (i = 0; i < words; i++) {
      wordsBuf[i] = spi->dev->data_buf[i]; // copy spi fifo to buffer
    }
    memcpy(out, bytesBuf, bytes); // copy buffer to output
  }
}

void espiTransferBytes(espi_t *spi, uint8_t *data, uint8_t *out, uint32_t size) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  while (size) {
    if (size > 64) {
      __espiTransferBytes(spi, data, out, 64);
      size -= 64;
      if (data) {
        data += 64;
      }
      if (out) {
        out += 64;
      }
    } else {
      __espiTransferBytes(spi, data, out, size);
      size = 0;
    }
  }
  SPI_MUTEX_UNLOCK();
}

void espiTransferBits(espi_t *spi, uint32_t data, uint32_t *out, uint8_t bits) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spiTransferBitsNL(spi, data, out, bits);
  SPI_MUTEX_UNLOCK();
}

  /*
   * Manual Lock Management
   * */

#define MSB_32_SET(var, val)                                                   \
  {                                                                            \
    uint8_t *d = (uint8_t *)&(val);                                            \
    (var) = d[3] | (d[2] << 8) | (d[1] << 16) | (d[0] << 24);                  \
  }
#define MSB_24_SET(var, val)                                                   \
  {                                                                            \
    uint8_t *d = (uint8_t *)&(val);                                            \
    (var) = d[2] | (d[1] << 8) | (d[0] << 16);                                 \
  }
#define MSB_16_SET(var, val)                                                   \
  { (var) = (((val)&0xFF00) >> 8) | (((val)&0xFF) << 8); }
#define MSB_PIX_SET(var, val)                                                  \
  {                                                                            \
    uint8_t *d = (uint8_t *)&(val);                                            \
    (var) = d[1] | (d[0] << 8) | (d[3] << 16) | (d[2] << 24);                  \
  }

void espiTransaction(espi_t *spi, uint32_t clockDiv, uint8_t dataMode,
                    uint8_t bitOrder) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
  spi->dev->clock.val = clockDiv;
  switch (dataMode) {
  case ESPI_MODE1:
    spi->dev->pin.ck_idle_edge = 0;
    spi->dev->user.ck_out_edge = 1;
    break;
  case ESPI_MODE2:
    spi->dev->pin.ck_idle_edge = 1;
    spi->dev->user.ck_out_edge = 1;
    break;
  case ESPI_MODE3:
    spi->dev->pin.ck_idle_edge = 1;
    spi->dev->user.ck_out_edge = 0;
    break;
  case ESPI_MODE0:
  default:
    spi->dev->pin.ck_idle_edge = 0;
    spi->dev->user.ck_out_edge = 0;
    break;
  }
  if (ESPI_MSBFIRST == bitOrder) {
    spi->dev->ctrl.wr_bit_order = 0;
    spi->dev->ctrl.rd_bit_order = 0;
  } else if (ESPI_LSBFIRST == bitOrder) {
    spi->dev->ctrl.wr_bit_order = 1;
    spi->dev->ctrl.rd_bit_order = 1;
  }
}

void espiSimpleTransaction(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_LOCK();
}

void espiEndTransaction(espi_t *spi) {
  if (!spi) {
    return;
  }
  SPI_MUTEX_UNLOCK();
}

void espiWriteByteNL(espi_t *spi, uint8_t data) {
  if (!spi) {
    return;
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
}

uint8_t espiTransferByteNL(espi_t *spi, uint8_t data) {
  if (!spi) {
    return 0;
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 7;
  spi->dev->miso_dlen.usr_miso_dbitlen = 7;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0] & 0xFF;
  return data;
}

void espiWriteShortNL(espi_t *spi, uint16_t data) {
  if (!spi) {
    return;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    MSB_16_SET(data, data);
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
}

uint16_t espiTransferShortNL(espi_t *spi, uint16_t data) {
  if (!spi) {
    return 0;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    MSB_16_SET(data, data);
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 15;
  spi->dev->miso_dlen.usr_miso_dbitlen = 15;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0] & 0xFFFF;
  if (!spi->dev->ctrl.rd_bit_order) {
    MSB_16_SET(data, data);
  }
  return data;
}

void espiWriteLongNL(espi_t *spi, uint32_t data) {
  if (!spi) {
    return;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    MSB_32_SET(data, data);
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 31;
  spi->dev->miso_dlen.usr_miso_dbitlen = 0;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
}

uint32_t espiTransferLongNL(espi_t *spi, uint32_t data) {
  if (!spi) {
    return 0;
  }
  if (!spi->dev->ctrl.wr_bit_order) {
    MSB_32_SET(data, data);
  }
  spi->dev->mosi_dlen.usr_mosi_dbitlen = 31;
  spi->dev->miso_dlen.usr_miso_dbitlen = 31;
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0];
  if (!spi->dev->ctrl.rd_bit_order) {
    MSB_32_SET(data, data);
  }
  return data;
}

void espiWriteNL(espi_t *spi, const void *data_in, size_t len) {
  size_t longs = len >> 2;
  if (len & 3) {
    longs++;
  }
  uint32_t *data = (uint32_t *)data_in;
  size_t c_len = 0, c_longs = 0;

  while (len) {
    c_len = (len > 64) ? 64 : len;
    c_longs = (longs > 16) ? 16 : longs;

    spi->dev->mosi_dlen.usr_mosi_dbitlen = (c_len * 8) - 1;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    for (int i = 0; i < c_longs; i++) {
      spi->dev->data_buf[i] = data[i];
    }
    spi->dev->cmd.usr = 1;
    while (spi->dev->cmd.usr)
      ;

    data += c_longs;
    longs -= c_longs;
    len -= c_len;
  }
}

void espiTransferBytesNL(espi_t *spi, const void *data_in, uint8_t *data_out,
                        size_t len) {
  if (!spi) {
    return;
  }
  size_t longs = len >> 2;
  if (len & 3) {
    longs++;
  }
  uint32_t *data = (uint32_t *)data_in;
  uint32_t *result = (uint32_t *)data_out;
  size_t c_len = 0, c_longs = 0;

  while (len) {
    c_len = (len > 64) ? 64 : len;
    c_longs = (longs > 16) ? 16 : longs;

    spi->dev->mosi_dlen.usr_mosi_dbitlen = (c_len * 8) - 1;
    spi->dev->miso_dlen.usr_miso_dbitlen = (c_len * 8) - 1;
    if (data) {
      for (int i = 0; i < c_longs; i++) {
        spi->dev->data_buf[i] = data[i];
      }
    } else {
      for (int i = 0; i < c_longs; i++) {
        spi->dev->data_buf[i] = 0xFFFFFFFF;
      }
    }
    spi->dev->cmd.usr = 1;
    while (spi->dev->cmd.usr)
      ;
    if (result) {
      for (int i = 0; i < c_longs; i++) {
        result[i] = spi->dev->data_buf[i];
      }
    }
    if (data) {
      data += c_longs;
    }
    if (result) {
      result += c_longs;
    }
    longs -= c_longs;
    len -= c_len;
  }
}

void espiTransferBitsNL(espi_t *spi, uint32_t data, uint32_t *out,
                       uint8_t bits) {
  if (!spi) {
    return;
  }

  if (bits > 32) {
    bits = 32;
  }
  uint32_t bytes = (bits + 7) / 8; // 64 max
  uint32_t mask = (((uint64_t)1 << bits) - 1) & 0xFFFFFFFF;
  data = data & mask;
  if (!spi->dev->ctrl.wr_bit_order) {
    if (bytes == 2) {
      MSB_16_SET(data, data);
    } else if (bytes == 3) {
      MSB_24_SET(data, data);
    } else {
      MSB_32_SET(data, data);
    }
  }

  spi->dev->mosi_dlen.usr_mosi_dbitlen = (bits - 1);
  spi->dev->miso_dlen.usr_miso_dbitlen = (bits - 1);
  spi->dev->data_buf[0] = data;
  spi->dev->cmd.usr = 1;
  while (spi->dev->cmd.usr)
    ;
  data = spi->dev->data_buf[0];
  if (out) {
    *out = data;
    if (!spi->dev->ctrl.rd_bit_order) {
      if (bytes == 2) {
        MSB_16_SET(*out, data);
      } else if (bytes == 3) {
        MSB_24_SET(*out, data);
      } else {
        MSB_32_SET(*out, data);
      }
    }
  }
}

void espiWritePixelsNL(espi_t *spi, const void *data_in, size_t len) {
  size_t longs = len >> 2;
  if (len & 3) {
    longs++;
  }
  bool msb = !spi->dev->ctrl.wr_bit_order;
  uint32_t *data = (uint32_t *)data_in;
  size_t c_len = 0, c_longs = 0, l_bytes = 0;

  while (len) {
    c_len = (len > 64) ? 64 : len;
    c_longs = (longs > 16) ? 16 : longs;
    l_bytes = (c_len & 3);

    spi->dev->mosi_dlen.usr_mosi_dbitlen = (c_len * 8) - 1;
    spi->dev->miso_dlen.usr_miso_dbitlen = 0;
    for (int i = 0; i < c_longs; i++) {
      if (msb) {
        if (l_bytes && i == (c_longs - 1)) {
          if (l_bytes == 2) {
            MSB_16_SET(spi->dev->data_buf[i], data[i]);
          } else {
            spi->dev->data_buf[i] = data[i] & 0xFF;
          }
        } else {
          MSB_PIX_SET(spi->dev->data_buf[i], data[i]);
        }
      } else {
        spi->dev->data_buf[i] = data[i];
      }
    }
    spi->dev->cmd.usr = 1;
    while (spi->dev->cmd.usr)
      ;

    data += c_longs;
    longs -= c_longs;
    len -= c_len;
  }
}

/*
 * Clock Calculators
 *
 * */

typedef union {
  uint32_t regValue;
  struct {
    unsigned regL : 6;
    unsigned regH : 6;
    unsigned regN : 6;
    unsigned regPre : 13;
    unsigned regEQU : 1;
  };
} spiClk_t;

#define ClkRegToFreq(reg)                                                      \
  (CPU_CLK_FREQ / (((reg)->regPre + 1) * ((reg)->regN + 1)))

uint32_t espiClockDivToFrequency(uint32_t clockDiv) {
  spiClk_t reg = {clockDiv};
  return ClkRegToFreq(&reg);
}

uint32_t espiFrequencyToClockDiv(uint32_t freq) {

  if (freq >= CPU_CLK_FREQ) {
    return SPI_CLK_EQU_SYSCLK;
  }

  const spiClk_t minFreqReg = {0x7FFFF000};
  uint32_t minFreq = ClkRegToFreq((spiClk_t *)&minFreqReg);
  if (freq < minFreq) {
    return minFreqReg.regValue;
  }

  uint8_t calN = 1;
  spiClk_t bestReg = {0};
  int32_t bestFreq = 0;

  while (calN <= 0x3F) {
    spiClk_t reg = {0};
    int32_t calFreq;
    int32_t calPre;
    int8_t calPreVari = -2;

    reg.regN = calN;

    while (calPreVari++ <= 1) {
      calPre = (((CPU_CLK_FREQ / (reg.regN + 1)) / freq) - 1) + calPreVari;
      if (calPre > 0x1FFF) {
        reg.regPre = 0x1FFF;
      } else if (calPre <= 0) {
        reg.regPre = 0;
      } else {
        reg.regPre = calPre;
      }
      reg.regL = ((reg.regN + 1) / 2);
      calFreq = ClkRegToFreq(&reg);
      if (calFreq == (int32_t)freq) {
        memcpy(&bestReg, &reg, sizeof(bestReg));
        break;
      } else if (calFreq < (int32_t)freq) {
        if (abs(freq - calFreq) < abs(freq - bestFreq)) {
          bestFreq = calFreq;
          memcpy(&bestReg, &reg, sizeof(bestReg));
        }
      }
    }
    if (calFreq == (int32_t)freq) {
      break;
    }
    calN++;
  }
  return bestReg.regValue;
}
