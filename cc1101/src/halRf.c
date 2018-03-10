#include "halRf.h"
#include "halSpi.h"
#include "spiwrap.h"

void halRfInit(void) { SPI_INIT(); }

//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacket(BYTE *txBuffer, UINT8 size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to
//      63 bytes. To use this function, GD00 must be configured to be asserted
//      when sync word is sent and de-asserted at the end of the packet =>
//      halSpiWriteReg(CC1101_IOCFG0, 0x06); The function implements polling of
//      GDO0. First it waits for GD00 to be set and then it waits for it to be
//      cleared.
//
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containing the data that are going to be
//          transmitted
//
//      UINT8 size
//          The size of the txBuffer
//-------------------------------------------------------------------------------------------------------
void halRfSendPacket(BYTE *txBuffer, UINT8 size) {

  halSpiWriteBurstReg(CC1101_TXFIFO, txBuffer, size);
  halSpiStrobe(CC1101_STX);

#if 0
  // Wait for GDO0 to be set -> sync transmitted
  while (!GDO0_PIN)
    ;

  // Wait for GDO0 to be cleared -> end of packet
  while (GDO0_PIN)
    ;
#endif
} // halRfSendPacket

//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION:
//      This function can be used to receive a packet of variable packet length
//      (first byte in the packet must be the length byte). The packet length
//      should not exceed the RX FIFO size. To use this function, GD00 must be
//      configured to be asserted when sync word is sent and de-asserted at the
//      end of the packet => halSpiWriteReg(CC1101_IOCFG0, 0x06); Also,
//      APPEND_STATUS in the PKTCTRL1 register must be enabled. The function
//      implements polling of GDO0. First it waits for GD00 to be set and then
//      it waits for it to be cleared. After the GDO0 pin has been de-asserted,
//      the RXBYTES register is read to make sure that there are bytes in the
//      FIFO. This is because the GDO signal will indicate sync received even if
//      the FIFO is flushed due to address filtering, CRC filtering, or packet
//      length filtering.
//
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the
//          incoming data should be stored. After this function returns, that
//          variable holds the packet length.
//
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to
//          filtering)
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length) {
  BYTE status[2];
  UINT8 packetLength;

  halSpiStrobe(CC1101_SRX);

#if 0
  // Wait for GDO0 to be set -> sync received
  while (!GDO0_PIN)
    ;

  // Wait for GDO0 to be cleared -> end of packet
  while (GDO0_PIN)
    ;
#endif

  // This status register is safe to read since it will not be updated after
  // the packet has been received (See the CC1100 and 2500 Errata Note)
  if ((halSpiReadStatus(CC1101_RXBYTES) & BYTES_IN_RXFIFO)) {

    // Read length byte
    packetLength = halSpiReadReg(CC1101_RXFIFO);

    // Read data from RX FIFO and store in rxBuffer
    if (packetLength <= *length) {
      halSpiReadBurstReg(CC1101_RXFIFO, rxBuffer, packetLength);
      *length = packetLength;

      // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
      halSpiReadBurstReg(CC1101_RXFIFO, status, 2);

      // MSB of LQI is the CRC_OK bit
      return (status[LQI] & CRC_OK);
    } else {
      *length = packetLength;

      // Make sure that the radio is in IDLE state before flushing the FIFO
      // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state
      // at this point)
      halSpiStrobe(CC1101_SIDLE);

      // Flush RX FIFO
      halSpiStrobe(CC1101_SFRX);
      return FALSE;
    }
  } else
    return FALSE;
} // halRfReceivePacket

//-------------------------------------------------------------------------------------------------------
//  BYTE spiGetTxStatus(void)
//
//  DESCRIPTION:
//  This function transmits a No Operation Strobe (SNOP) to get the status of
//  the radio.
// Status byte:
// ---------------------------------------------------------------------------
//  |          |            |                                                 |
//  | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (free bytes in the TX FIFO |
//  |          |            |                                                 |
//  ---------------------------------------------------------------------------
// STATE[2:0]:
// Value | State
//  --------------------------
//  000   | Idle
//  001   | RX
//  010   | TX
//  011   | FSTXON
//  100   | CALIBRATE
//  101   | SETTLING
//  110   | RXFIFO_OVERFLOW
//  111   | TX_FIFO_UNDERFLOW
//-------------------------------------------------------------------------------------------------------
BYTE spiGetTxStatus(void) {} // spiGetTxStatus

//-------------------------------------------------------------------------------------------------------
//  BYTE spiGetRxStatus(void)
//
//  DESCRIPTION:
//  This function transmits a No Operation Strobe (SNOP) with the read bit set
//  to get the status of the radio.
// Status byte:
// --------------------------------------------------------------------------------
//  |          |            |                                                  |
//  | CHIP_RDY | STATE[2:0] | FIFO_BYTES_AVAILABLE (avail bytes in the RX FIFO |
//  |          |            |                                                  |
//  --------------------------------------------------------------------------------
// STATE[2:0]:
// Value | State
//  --------------------------
//  000   | Idle
//  001   | RX
//  010   | TX
//  011   | FSTXON
//  100   | CALIBRATE
//  101   | SETTLING
//  110   | RXFIFO_OVERFLOW
//  111   | TX_FIFO_UNDERFLOW
//-------------------------------------------------------------------------------------------------------
BYTE spiGetRxStatus(void) {} // spiGetRxStatus
