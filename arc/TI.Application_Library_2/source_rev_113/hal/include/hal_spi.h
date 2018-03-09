/***********************************************************************************
    Filename: hal_spi.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_SPI_H
#define HAL_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_types.h>

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void  halSpiInit(uint8 clockrate);
uint8 halSpiRead(uint8 addr, uint8* data, uint16 len);
uint8 halSpiWrite(uint8 addr, const uint8* data, uint16 len);
uint8 halSpiStrobe(uint8 cmd);


#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif
