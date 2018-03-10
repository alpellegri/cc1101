#include "halSpi.h"
#include "halRf.h"

//-------------------------------------------------------------------------------------------------------
//  void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count) {
  UINT8 i;
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

//-------------------------------------------------------------------------------------------------------
//  BYTE halSpiReadReg(BYTE addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
UINT8 halSpiReadReg(BYTE addr) {
  UINT8 x;
  SPI_BEGIN();
  SPI_TX(addr | READ_BURST);
  SPI_WAIT();
  SPI_TX(0);
  SPI_WAIT();
  x = SPI_RX();
  SPI_END();
  return x;
} // halSpiReadReg

//-------------------------------------------------------------------------------------------------------
//  BYTE halSpiReadStatus(BYTE addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 status register.
//-------------------------------------------------------------------------------------------------------
UINT8 halSpiReadStatus(BYTE addr) {
  UINT8 x;
  SPI_BEGIN();
  SPI_TX(addr | READ_BURST);
  SPI_WAIT();
  SPI_TX(0);
  SPI_WAIT();
  x = SPI_RX();
  SPI_END();
  return x;
} // halSpiReadStatus

//-------------------------------------------------------------------------------------------------------
//  void halSpiStrobe(BYTE strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      BYTE strobe
//          Strobe command
//-------------------------------------------------------------------------------------------------------
void halSpiStrobe(BYTE strobe) {
  SPI_BEGIN();
  SPI_TX(strobe);
  SPI_WAIT();
  SPI_END();
} // halSpiStrobe

//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteReg(BYTE addr, BYTE value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      BYTE addr
//          Address of a specific CCxxx0 register to accessed.
//      BYTE value
//          Value to be written to the specified CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
void halSpiWriteReg(BYTE addr, BYTE value) {
  SPI_BEGIN();
  SPI_TX(addr);
  SPI_WAIT();
  SPI_TX(value);
  SPI_WAIT();
  SPI_END();
} // halSpiWriteReg

//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst
//      access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count) {
  UINT8 i;
  SPI_BEGIN();
  SPI_TX(addr | WRITE_BURST);
  SPI_WAIT();
  for (i = 0; i < count; i++) {
    SPI_TX(buffer[i]);
    SPI_WAIT();
  }
  SPI_END();
} // halSpiWriteBurstReg
