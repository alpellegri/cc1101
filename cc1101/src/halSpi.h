#ifndef HALSPI_H
#define HALSPI_H

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

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
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count);

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
UINT8 halSpiReadReg(BYTE addr);

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
UINT8 halSpiReadStatus(BYTE addr);

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
void halSpiStrobe(BYTE strobe);

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
void halSpiWriteReg(BYTE addr, BYTE value);

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
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count);

#ifdef __cplusplus
}
#endif

#endif
