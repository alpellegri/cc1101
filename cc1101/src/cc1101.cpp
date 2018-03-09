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

#include "cc1101.h"

/******************************************************************************
 * LOCAL VARIABLES
 */

/******************************************************************************
 * STATIC FUNCTIONS
 */
static void trxReadWriteBurstSingle(uint8_t addr, uint8_t *pData, uint16_t len);

uint8_t SPI_Rx(void) { return (uint8_t)(SPI1W0 & 0xff); }

/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *
 * @param       input, output parameters
 *
 * @return      describe return value, if any
 */
void exp430RfSpiInit(void) {}

/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *
 * @param       input, output parameters
 *
 * @return      describe return value, if any
 */
uint8_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData,
                         uint16_t len) {
  uint8_t readValue;

  // Pull CS_N low and wait for SO to go low before communication starts
  SPI_BEGIN();
  SPI_WAIT_MISO();
  // send register address byte
  SPI_TX(accessType | addrByte);
  SPI_WAIT_DONE();
  // Storing chip status
  readValue = SPI_RX();

  trxReadWriteBurstSingle(accessType | addrByte, pData, len);
  SPI_END();
  // return the status byte value */
  return (readValue);
}

/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
rfStatus_t trxSpiCmdStrobe(uint8_t cmd) {
  uint8_t rc;
  SPI_BEGIN();
  SPI_WAIT_MISO();
  SPI_TX(cmd);
  SPI_WAIT_DONE();
  rc = SPI_RX();
  SPI_END();
  return (rc);
}

/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes
 * to the radio in burst mode or read len bytes from the radio in burst mode if
 * the burst bit is set. If the burst bit is not set, only one data
 * byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              SPI_BEGIN();
 *              while(TRXEM_PORT_IN & SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t
 * len); SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
static void trxReadWriteBurstSingle(uint8_t addr, uint8_t *pData,
                                    uint16_t len) {
  uint16_t i;
  /* Communicate len number of bytes: if RX - the procedure sends 0x00 to push
   * bytes from slave*/
  if (addr & RADIO_READ_ACCESS) {
    if (addr & RADIO_BURST_ACCESS) {
      for (i = 0; i < len; i++) {
        SPI_TX(0); /* Possible to combining read and write as one access type */
        SPI_WAIT_DONE();
        *pData = SPI_RX(); /* Store pData from last pData RX */
        pData++;
      }
    } else {
      SPI_TX(0);
      SPI_WAIT_DONE();
      *pData = SPI_RX();
    }
  } else {
    if (addr & RADIO_BURST_ACCESS) {
      /* Communicate len number of bytes: if TX - the procedure doesn't
       * overwrite pData */
      for (i = 0; i < len; i++) {
        SPI_TX(*pData);
        SPI_WAIT_DONE();
        pData++;
      }
    } else {
      SPI_TX(*pData);
      SPI_WAIT_DONE();
    }
  }
  return;
}
