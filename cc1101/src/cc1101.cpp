#include <Arduino.h>
#include <SPI.h>

#include "cc1101.h"

#define PORT_GDO0 5
#define byte uint8_t

uint8_t SPI_Rx(void) { return (uint8_t)(SPI1W0 & 0xff); }

void CTRL_WAIT_SYNC(uint8_t pin) {
  while (digitalRead(pin) == false) {
    delay(1);
  }
}

void CTRL_WAIT_EOT(uint8_t pin) {
  while (digitalRead(pin) == true) {
    delay(1);
  }
}

void ICACHE_RAM_ATTR SPI_BEGIN(void) { digitalWrite(SS, LOW); }
void ICACHE_RAM_ATTR SPI_WAIT(void) { delay(1); }
void ICACHE_RAM_ATTR SPI_WAIT_MISO(void) {}
void ICACHE_RAM_ATTR SPI_WAIT_DONE(void) {}
void ICACHE_RAM_ATTR SPI_TX(uint8_t x) { SPI.transfer(x); }
uint8_t ICACHE_RAM_ATTR SPI_RX(void) { return SPI_Rx(); }
void ICACHE_RAM_ATTR SPI_END(void) { digitalWrite(SS, HIGH); }

void (*cb1)(void);

/* Handle interrupt from CC1101 (INT0) gdo0 on pin2 */
uint8_t cnt;
void ICACHE_RAM_ATTR Interrupt1(void) {
  Serial.println(cnt++);
  cb1();
}

void SetCb1(uint8_t pin, void (*cb)(void)) {
  cb1 = cb;
  attachInterrupt(pin, Interrupt1, FALLING);
}

/******************************************************************************
 * @fn          function name
 *
 * @brief       Description of the function
 *
 * @param       input, output parameters
 *
 * @return      describe return value, if any
 */
void SPI_INIT(void) {
  // SPI.setHwCs(SS);
  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();
  SPI.setFrequency(1000000);
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
}

//------------------------------------------------------------------------------
//  void readBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      uint8_t addr
//          Address of the first CCxxx0 register to be accessed.
//      uint8_t *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      uint8_t count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//------------------------------------------------------------------------------
void ICACHE_RAM_ATTR CC1101::readBurstReg(uint8_t addr, uint8_t *buffer,
                                                uint8_t count) {
  uint8_t i;
  SPI_BEGIN();
  SPI_TX(addr | READ_BURST);
  SPI_WAIT();
  for (i = 0; i < count; i++) {
    SPI_TX(0);
    SPI_WAIT();
    buffer[i] = SPI_RX();
  }
  SPI_END();
} // halSpiReadBurstReg

//------------------------------------------------------------------------------
//  uint8_t readReg(uint8_t addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      uint8_t addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      uint8_t
//          Value of the accessed CCxxx0 register.
//------------------------------------------------------------------------------
uint8_t ICACHE_RAM_ATTR CC1101::readReg(uint8_t addr) {
  uint8_t x;
  SPI_BEGIN();
  SPI_TX(addr | READ_BURST);
  SPI_WAIT();
  SPI_TX(0);
  SPI_WAIT();
  x = SPI_RX();
  SPI_END();
  return x;
} // halSpiReadReg

//------------------------------------------------------------------------------
//  uint8_t readStatus(uint8_t addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      uint8_t addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      uint8_t
//          Value of the accessed CCxxx0 status register.
//------------------------------------------------------------------------------
uint8_t ICACHE_RAM_ATTR CC1101::readStatus(uint8_t addr) {
  uint8_t x;
  SPI_BEGIN();
  SPI_TX(addr | READ_BURST);
  SPI_WAIT();
  SPI_TX(0);
  SPI_WAIT();
  x = SPI_RX();
  SPI_END();
  return x;
} // halSpiReadStatus

//------------------------------------------------------------------------------
//  void strobe(uint8_t strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      uint8_t strobe
//          Strobe command
//------------------------------------------------------------------------------
void ICACHE_RAM_ATTR CC1101::strobe(uint8_t strobe) {
  SPI_BEGIN();
  SPI_TX(strobe);
  SPI_WAIT();
  SPI_END();
} // halSpiStrobe

//------------------------------------------------------------------------------
//  void writeReg(uint8_t addr, uint8_t value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      uint8_t addr
//          Address of a specific CCxxx0 register to accessed.
//      uint8_t value
//          Value to be written to the specified CCxxx0 register.
//------------------------------------------------------------------------------
void ICACHE_RAM_ATTR CC1101::writeReg(uint8_t addr, uint8_t value) {
  SPI_BEGIN();
  SPI_TX(addr);
  SPI_WAIT();
  SPI_TX(value);
  SPI_WAIT();
  SPI_END();
} // halSpiWriteReg

//------------------------------------------------------------------------------
//  void writeBurstReg(uint8_t addr, uint8_t *buffer, uint8_t count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst
//      access.
//
//  ARGUMENTS:
//      uint8_t addr
//          Address of the first CCxxx0 register to be accessed.
//      uint8_t *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      uint8_t count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//------------------------------------------------------------------------------
void ICACHE_RAM_ATTR CC1101::writeBurstReg(uint8_t addr, uint8_t *buffer,
                                           uint8_t count) {
  uint8_t i;
  SPI_BEGIN();
  SPI_TX(addr | WRITE_BURST);
  SPI_WAIT();
  for (i = 0; i < count; i++) {
    SPI_TX(buffer[i]);
    SPI_WAIT();
  }
  SPI_END();
} // halSpiWriteBurstReg

//------------------------------------------------------------------------------
//  uint8_t getStatus(void)
//
//  DESCRIPTION:
//  This function transmits a No Operation Strobe (SNOP) to get the status of
//  the radio.
// Status byte:
// ---------------------------------------------------------------------------
//  |          |            |                                                 |
//  | CHIP_RDY | STATE[2:0] | FIFO_uint8_tS_AVAIL (free bytes in the TX FIFO  |
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
//------------------------------------------------------------------------------
uint8_t ICACHE_RAM_ATTR CC1101::getStatus(void) {
  uint8_t x;
  SPI_BEGIN();
  SPI_TX(CC1101_SNOP | READ_BURST);
  SPI_WAIT();
  x = SPI_RX();
  SPI_END();
} // spiGetTxStatus

/**
 * CC1101
 *
 * Class constructor
 */
CC1101::CC1101(void) {
  SPI_INIT(); // Initialize SPI interface
}

/**
 * init
 *
 * Initialize CC1101 radio
 *
 * @param freq Carrier frequency
 */
void CC1101::init(void) {
  uint8_t writeByte;
#ifdef PA_TABLE
  uint8_t paTable[] = PA_TABLE;
#endif

  // reset radio
  strobe(CC1101_SRES);
  // write registers to radio
  for (uint16_t i = 0; i < preferredSettings_size; i++) {
    writeByte = preferredSettings[i].data;
    // halSpiWriteBurstReg(preferredSettings[i].addr, &writeByte, 1);
    writeReg(preferredSettings[i].addr, writeByte);
  }
#ifdef PA_TABLE
  // write PA_TABLE
  writeBurstReg(CC1101_PATABLE, paTable, sizeof(paTable));
#endif
}

//------------------------------------------------------------------------------
//  void send(uint8_t *txBuffer, uint8_t size)
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
//      uint8_t *txBuffer
//          Pointer to a buffer containing the data that are going to be
//          transmitted
//
//      uint8_t size
//          The size of the txBuffer
//------------------------------------------------------------------------------
void CC1101::send(uint8_t *txBuffer, uint8_t size) {

  writeBurstReg(CC1101_TXFIFO, txBuffer, size);
  strobe(CC1101_STX);

  // Wait for GDO0 to be set -> sync transmitted
  CTRL_WAIT_SYNC(PORT_GDO0);

  // Wait for GDO0 to be cleared -> end of packet
  CTRL_WAIT_EOT(PORT_GDO0);
} // halRfSendPacket

//------------------------------------------------------------------------------
//  BOOL receive(uint8_t *rxBuffer, uint8_t *length)
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
//      the RXuint8_tS register is read to make sure that there are bytes in the
//      FIFO. This is because the GDO signal will indicate sync received even if
//      the FIFO is flushed due to address filtering, CRC filtering, or packet
//      length filtering.
//
//  ARGUMENTS:
//      uint8_t *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      uint8_t *length
//          Pointer to a variable containing the size of the buffer where the
//          incoming data should be stored. After this function returns, that
//          variable holds the packet length.
//
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to
//          filtering)
//------------------------------------------------------------------------------
bool CC1101::receive(uint8_t *rxBuffer, uint8_t *length) {
  uint8_t status[2];
  uint8_t packetLength;

  strobe(CC1101_SRX);

  // Wait for GDO0 to be set -> sync transmitted
  CTRL_WAIT_SYNC(PORT_GDO0);

  // Wait for GDO0 to be cleared -> end of packet
  CTRL_WAIT_EOT(PORT_GDO0);

  // This status register is safe to read since it will not be updated after
  // the packet has been received (See the CC1100 and 2500 Errata Note)
  if ((readStatus(CC1101_RXBYTES) & uint8_tS_IN_RXFIFO)) {

    // Read length byte
    packetLength = readReg(CC1101_RXFIFO);

    // Read data from RX FIFO and store in rxBuffer
    if (packetLength <= *length) {
      readBurstReg(CC1101_RXFIFO, rxBuffer, packetLength);
      *length = packetLength;

#if 0
      // Read the 2 appended status bytes (status[0] = RSSI, status[1] = LQI)
      halSpiReadBurstReg(CC1101_RXFIFO, status, 2);

      // MSB of LQI is the CRC_OK bit
      return (status[LQI] & CRC_OK);
#else
      return true;
#endif
    } else {
      *length = packetLength;

      // Make sure that the radio is in IDLE state before flushing the FIFO
      // (Unless RXOFF_MODE has been changed, the radio should be in IDLE state
      // at this point)
      strobe(CC1101_SIDLE);

      // Flush RX FIFO
      strobe(CC1101_SFRX);
      return true;
    }
  } else
    return true;
} // halRfReceivePacket
