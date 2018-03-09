/**************************************************************************/ /**
     @file       file name

     @brief      describtion

 ******************************************************************************/
#ifndef CC1101_H
#define CC1101_H

/******************************************************************************
 * INCLUDES
 */

#include <Arduino.h>

/******************************************************************************
 * CONSTANTS
 */

#define RADIO_BURST_ACCESS 0x40
#define RADIO_SINGLE_ACCESS 0x00
#define RADIO_READ_ACCESS 0x80
#define RADIO_WRITE_ACCESS 0x00

/******************************************************************************
 * MACROS
 */

#define SPI_BEGIN()                                                            \
  do {                                                                         \
    SPI.begin();                                                               \
  } while (0);
#define SPI_WAIT_MISO()                                                        \
  do {                                                                         \
    delay(10);                                                                 \
  \
} while (0);
#define SPI_TX(x)                                                              \
  do {                                                                         \
    SPI.transfer(x);                                                           \
  } while (0);
#define SPI_WAIT_DONE()
#define SPI_RX() SPI_Rx()
#define SPI_END()                                                              \
  do {                                                                         \
    SPI.end();                                                                 \
  } while (0);

/******************************************************************************
 * TYPEDEFS
 */

typedef uint8_t rfStatus_t;

/******************************************************************************
 * PROTOTYPES
 */
void exp430RfSpiInit(void);

uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData,
                         uint16_t len);
rfStatus_t trxSpiCmdStrobe(uint8_t cmd);

uint8_t SPI_Rx(void);

#endif
