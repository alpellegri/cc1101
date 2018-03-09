/******************************************************************************
    Filename: cc1101_spi.c

    Description: implementation file for a minimum set of neccessary functions
                 to communicate with CC1101 over SPI

*******************************************************************************/

/******************************************************************************
 * INCLUDES
 */

#include <Arduino.h>

#include "cc1101_spi.h"

/******************************************************************************
 * @fn          cc1101SpiReadReg
 *
 * @brief       Reads register(s). If len  = 1: one byte is read
 *                                 if len != 1: len bytes are read in burst
 *                                              mode
 * input parameters
 *
 * @param       addr   - address to read
 * @param       *pData - pointer to data array where read data is stored
 * @param       len    - numbers of bytes to read starting from addr
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t cc1101SpiReadReg(uint8_t addr, uint8_t *pData, uint8_t len) {
  uint8_t rc;
  rc = trx8BitRegAccess((RADIO_BURST_ACCESS | RADIO_READ_ACCESS), addr, pData,
                        len);
  return (rc);
}

/******************************************************************************
 * @fn          cc1101SpiWriteReg
 *
 * @brief       writes register(s). If len  = 1: one byte is written
 *                                  if len != 1: len bytes at *data is written
 *                                               in burst mode.
 * input parameters
 *
 * @param       addr   - register address to write
 * @param       *pData - pointer to data array that is written
 * @param       len    - number of bytes written
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t cc1101SpiWriteReg(uint8_t addr, uint8_t *pData, uint8_t len) {
  uint8_t rc;
  rc = trx8BitRegAccess((RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS), addr, pData,
                        len);
  return (rc);
}

/******************************************************************************
 * @fn          cc1101SpiWriteTxFifo
 *
 * @brief       Writes provided data to TX FIFO
 *
 * input parameters
 *
 * @param       *pData - pointer to data array that is written to TX FIFO
 * @param       len    - Length of data array to be written
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t cc1101SpiWriteTxFifo(uint8_t *pData, uint8_t len) {
  uint8_t rc;
  rc = trx8BitRegAccess((RADIO_BURST_ACCESS | RADIO_WRITE_ACCESS), CC1101_FIFO,
                        pData, len);
  return (rc);
}

/******************************************************************************
 * @fn          cc1101SpiReadRxFifo
 *
 * @brief       Reads RX FIFO values to pData array
 *
 * input parameters
 *
 * @param       *pData - pointer to data array where RX FIFO bytes are saved
 * @param       len    - number of bytes to read from the RX FIFO
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t cc1101SpiReadRxFifo(uint8_t *pData, uint8_t len) {
  uint8_t rc;
  rc = trx8BitRegAccess((RADIO_BURST_ACCESS | RADIO_READ_ACCESS), CC1101_FIFO,
                        pData, len);
  return (rc);
}

/******************************************************************************
 * @fn      cc1101GetTxStatus(void)
 *
 * @brief   This function transmits a No Operation Strobe (SNOP) to get the
 *          status of the radio and the number of free bytes in the TX FIFO.
 *
 *          Status byte:
 *
 *          ---------------------------------------------------------------------------
 *          |          |            | | | CHIP_RDY | STATE[2:0] |
 * FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO | |          |
 * |                                                 |
 *          ---------------------------------------------------------------------------
 *
 *          NOTE:
 *          When reading a status register over the SPI interface while the
 *          register is updated by the radio hardware, there is a small, but
 *          finite, probability that the result is corrupt. This also applies
 *          to the chip status byte. The CC1100 and CC1101 errata notes explain
 *          the problem and propose several work arounds.
 *
 * input parameters
 *
 * @param   none
 *
 * output parameters
 *
 * @return  rfStatus_t
 *
 */
rfStatus_t cc1101GetTxStatus(void) { return (trxSpiCmdStrobe(CC1101_SNOP)); }

/******************************************************************************
 *
 *  @fn       cc1101GetRxStatus(void)
 *
 *  @brief
 *            This function transmits a No Operation Strobe (SNOP) with the
 *            read bit set to get the status of the radio and the number of
 *            available bytes in the RXFIFO.
 *
 *            Status byte:
 *
 *            --------------------------------------------------------------------------------
 *            |          |            | | | CHIP_RDY | STATE[2:0] |
 * FIFO_BYTES_AVAILABLE (available bytes in the RX FIFO | |
 * |            | |
 *            --------------------------------------------------------------------------------
 *
 *            NOTE:
 *            When reading a status register over the SPI interface while the
 *            register is updated by the radio hardware, there is a small, but
 *            finite, probability that the result is corrupt. This also applies
 *            to the chip status byte. The CC1100 and CC1101 errata notes
 * explain the problem and propose several work arounds.
 *
 * input parameters
 *
 * @param     none
 *
 * output parameters
 *
 * @return    rfStatus_t
 *
 */
rfStatus_t cc1101GetRxStatus(void) {
  return (trxSpiCmdStrobe(CC1101_SNOP | RADIO_READ_ACCESS));
}
