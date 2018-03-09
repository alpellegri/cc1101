/******************************************************************************
    Filename: hal_spi.c

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include "hal_types.h"
#include "hal_defs.h"
#include "hal_board.h"
#include "hal_spi.h"
#include "hal_spi_config.h"



//----------------------------------------------------------------------------------
//   Target specific initialization of SPI interface in hal_spi_config.c
//----------------------------------------------------------------------------------



//----------------------------------------------------------------------------------
//  void halSpiWrite(uint8 addr, const uint8 *buffer, uint16 length)
//
//  DESCRIPTION:
//    Write data to device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is written. For single byte write, set length to 1.
//----------------------------------------------------------------------------------
uint8 halSpiWrite(uint8 addr, const uint8* data, uint16 length)
{
    uint16 i;
    uint8 rc;

    HAL_SPI_BEGIN;
#ifdef HAL_SPI_INTERFACE_BITBANG
    rc = halSpiBitbangOut(addr);
    for (i = 0; i < length; i++)
    {
        halSpiBitbangOut(data[i]);
    }
#else
    HAL_SPI_TXBUF_SET(addr);
    HAL_SPI_WAIT_TXFIN;
    rc = HAL_SPI_RXBUF;
    for (i = 0; i < length; i++)
    {
      HAL_SPI_TXBUF_SET(data[i]);
      HAL_SPI_WAIT_TXBUF;
    }
#ifndef HAL_SPI_INTERFACE_USI
    HAL_SPI_WAIT_TXFIN;
#endif
#endif
    HAL_SPI_END;
    return(rc);

}

//----------------------------------------------------------------------------------
//  uint8 halSpiRead(uint8 addr, uint8* data, uint16 length)
//
//  DESCRIPTION:
//    Read data from device, starting at internal device address "addr".
//    The device will increment the address internally for every new byte
//    that is read. Note that the master device needs to write a dummy byte
//    (in this case 0) for every new byte in order to generate the clock to
//    clock out the data. For single byte read, set length to 1.
//----------------------------------------------------------------------------------
uint8 halSpiRead(uint8 addr, uint8* data, uint16 length)
{
    uint16 i;
    uint8 rc;

    HAL_SPI_BEGIN;
#ifdef HAL_SPI_INTERFACE_BITBANG
    rc = halSpiBitbangOut(addr);
    for (i = 0; i < length; i++)
    {
        data[i] = halSpiBitbangIn();
    }
#else
    HAL_SPI_TXBUF_SET(addr);
    HAL_SPI_WAIT_TXFIN;
    rc = HAL_SPI_RXBUF;
    for (i = 0; i < length; i++)
    {
        HAL_SPI_TXBUF_SET(0);        // Dummy write to read data byte
        HAL_SPI_WAIT_TXFIN;
        data[i] = HAL_SPI_RXBUF;     // Store data from last data RX
    }
#endif
    HAL_SPI_END;
    return(rc);
}


//----------------------------------------------------------------------------------
//  uint8 halSpiStrobe(uint8 cmd)
//
//  DESCRIPTION:
//    Special write function, writing only one byte (cmd) to the device.
//----------------------------------------------------------------------------------
uint8 halSpiStrobe(uint8 cmd)
{
    uint8 rc;

    HAL_SPI_BEGIN;
#ifdef HAL_SPI_INTERFACE_BITBANG
    rc = halSpiBitbangOut(cmd);
#else
    HAL_SPI_TXBUF_SET(cmd);
    HAL_SPI_WAIT_TXFIN;
    rc = HAL_SPI_RXBUF;
#endif
    HAL_SPI_END;
    return(rc);
}

