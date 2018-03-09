/******************************************************************************
    Filename: hal_spi_config.h

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#ifndef HAL_SPI_CONFIG_H
#define HAL_SPI_CONFIG_H

#include "hal_types.h"
#include "hal_defs.h"
#include "hal_board.h"
#include "hal_spi.h"


//----------------------------------------------------------------------------------
//   Generic SPI plugin module. Supports all known MSP430 SPI interfaces.
//----------------------------------------------------------------------------------




//----------------------------------------------------------------------------------
//   Common Macros
//----------------------------------------------------------------------------------
#define HAL_SPI_CS_DEASSERT  MCU_IO_SET(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN)
#define HAL_SPI_CS_ASSERT    MCU_IO_CLR(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN)
#define HAL_SPI_SOMI_VAL     MCU_IO_GET(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN)
#define HAL_SPI_BEGIN        st( HAL_SPI_CS_ASSERT; while(HAL_SPI_SOMI_VAL); )
#define HAL_SPI_END          st( HAL_SPI_CS_DEASSERT; )


//----------------------------------------------------------------------------------
//  USART0
//----------------------------------------------------------------------------------
#if defined HAL_SPI_INTERFACE_USART0

#define HAL_SPI_WAIT_RXFIN   st( while(!(IFG1 & URXIFG0)); )
#define HAL_SPI_WAIT_TXFIN   st( while(!(U0TCTL & TXEPT)); )
#define HAL_SPI_WAIT_TXBUF   st( while(!(IFG1 & UTXIFG0)); )
#define HAL_SPI_RXBUF        U0RXBUF
#define HAL_SPI_TXBUF        U0TXBUF
#define HAL_SPI_TXBUF_SET(x) st( HAL_SPI_TXBUF = (x); )


//----------------------------------------------------------------------------------
//  USART1
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USART1

#define HAL_SPI_WAIT_RXFIN   st( while(!(IFG2 & URXIFG1)); )
#define HAL_SPI_WAIT_TXFIN   st( while(!(U1TCTL & TXEPT)); )
#define HAL_SPI_WAIT_TXBUF   st( while(!(IFG2 & UTXIFG1)); )
#define HAL_SPI_RXBUF        U1RXBUF
#define HAL_SPI_TXBUF        U1TXBUF
#define HAL_SPI_TXBUF_SET(x) st( HAL_SPI_TXBUF = (x); )


//----------------------------------------------------------------------------------
//  USCIA0
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USCIA0

#define HAL_SPI_WAIT_RXFIN   st( while(!(IFG2 & UCA0RXIFG)); )
#define HAL_SPI_WAIT_TXFIN   st( while(UCA0STAT & UCBUSY); )
#define HAL_SPI_WAIT_TXBUF   st( while(!(IFG2 & UCA0TXIFG)); )
#define HAL_SPI_RXBUF        UCA0RXBUF
#define HAL_SPI_TXBUF        UCA0TXBUF
#define HAL_SPI_TXBUF_SET(x) st( HAL_SPI_TXBUF = (x); )


//----------------------------------------------------------------------------------
//  USCIB0
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USCIB0

#define HAL_SPI_WAIT_RXFIN   st( while(!(IFG2 & UCB0RXIFG)); )
#define HAL_SPI_WAIT_TXFIN   st( while(UCB0STAT & UCBUSY); )
#define HAL_SPI_WAIT_TXBUF   st( while(!(IFG2 & UCB0TXIFG)); )
#define HAL_SPI_RXBUF        UCB0RXBUF
#define HAL_SPI_TXBUF        UCB0TXBUF
#define HAL_SPI_TXBUF_SET(x) st( HAL_SPI_TXBUF = (x); )


//----------------------------------------------------------------------------------
//  USI
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USI

#define HAL_SPI_WAIT_RXFIN   st( USICNT = 8; while(!(USICTL1&USIIFG)); )
#define HAL_SPI_WAIT_TXFIN   st( USICNT = 8; while(!(USICTL1&USIIFG)); )
#define HAL_SPI_WAIT_TXBUF   st( USICNT = 8; while(!(USICTL1&USIIFG)); )
#define HAL_SPI_RXBUF        USISRL
#define HAL_SPI_TXBUF        USISRL
#define HAL_SPI_TXBUF_SET(x) st( HAL_SPI_TXBUF = (x); )


//----------------------------------------------------------------------------------
//  BITBANG
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_BITBANG

uint8 halSpiBitbangOut(uint8 data);
uint8 halSpiBitbangIn(void);

#define HAL_SPI_WAIT_RXFIN   asm("   nop")
#define HAL_SPI_WAIT_TXFIN   asm("   nop")
#define HAL_SPI_WAIT_TXBUF   asm("   nop")
#define HAL_SPI_RXBUF
#define HAL_SPI_TXBUF
#define HAL_SPI_TXBUF_SET(x) halSpiBitbangOut((x))


//----------------------------------------------------------------------------------
//  Unknown interface
//----------------------------------------------------------------------------------
#else
#error "You must select an SPI interface (USART/USCI/USI/BITBANG)"
#endif

#endif
