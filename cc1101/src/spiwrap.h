/**************************************************************************/ /**
     @file       file name

     @brief      describtion

 ******************************************************************************/
#ifndef SPIWRAP_H
#define SPIWRAP_H

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

void SPI_BEGIN(void);
void SPI_WAIT(void);
void SPI_WAIT_MISO(void);
void SPI_WAIT_DONE(void);
void SPI_TX(BYTE x);
BYTE SPI_RX(void);
void SPI_END(void);
void SPI_INIT(void);

#ifdef __cplusplus
}
#endif

#endif
