/******************************************************************************
    Filename: hal_spi_config.c

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include "hal_types.h"
#include "hal_defs.h"
#include "hal_board.h"
#include "hal_spi.h"
#include "hal_spi_config.h"


//----------------------------------------------------------------------------------
//   Generic SPI plugin module. Supports all known MSP430 SPI interfaces.
//----------------------------------------------------------------------------------



//----------------------------------------------------------------------------------
//  USART0
//----------------------------------------------------------------------------------
#if defined HAL_SPI_INTERFACE_USART0

void halSpiInit(uint8 clockrate)
{
    // Keep peripheral in reset state
    U0CTL = SWRST;

    // 8-bit SPI Master 3-pin mode, with SMCLK as clock source
    U0CTL  |= CHAR + SYNC + MM;
    U0TCTL |= CKPH + SSEL1 + SSEL0 + STC;

    // Ignore clockrate argument for now, just use clock source/2
    U0BR0  = 0x02;
    U0BR1  = 0x00;
    U0MCTL = 0x00;

    // Enable SPI mode
    ME1 |= USPIE0;

    // Set up pins used by peripheral unit
    MCU_IO_PERIPHERAL(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
    MCU_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);

    // Release for operation
    U0CTL &= ~SWRST;
}

//----------------------------------------------------------------------------------
//  USART1
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USART1

void halSpiInit(uint8 clockrate)
{
    // Keep peripheral in reset state
    U1CTL = SWRST;

    // 8-bit SPI Master 3-pin mode, with SMCLK as clock source
    U1CTL  |= CHAR + SYNC + MM;
    U1TCTL |= CKPH + SSEL1 + SSEL0 + STC;

    // Ignore clockrate argument for now, just use clock source/2
    U1BR0  = 0x02;
    U1BR1  = 0x00;
    U1MCTL = 0x00;

    // Enable SPI mode
    ME2 |= USPIE1;

    // Set up pins used by peripheral unit
    MCU_IO_PERIPHERAL(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
    MCU_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);

    // Release for operation
    U1CTL &= ~SWRST;
}

//----------------------------------------------------------------------------------
//  USCIA0
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USCIA0

void halSpiInit(uint8 clockrate)
{
    // Keep peripheral in reset state
    UCA0CTL1 = UCSWRST;

    // 8-bit SPI Master 3-pin mode, with SMCLK as clock source
    UCA0CTL0  = UCCKPH + UCMSB + UCMST + UCSYNC;
    UCA0CTL1 |= UCSSEL1 + UCSSEL0;

    // Ignore clockrate argument for now, just use clock source/2
    UCA0BR0 = 0x02;
    UCA0BR1 = 0x00;

    // Set up pins used by peripheral unit
    MCU_IO_PERIPHERAL(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
    MCU_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);

    // Release for operation
    UCA0CTL1 &= ~UCSWRST;
}

//----------------------------------------------------------------------------------
//  USCIB0
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USCIB0

void halSpiInit(uint8 clockrate)
{
    // Keep peripheral in reset state
    UCB0CTL1 = UCSWRST;

    // 8-bit SPI Master 3-pin mode, with SMCLK as clock source
    UCB0CTL0  = UCCKPH + UCMSB + UCMST + UCSYNC;
    UCB0CTL1 |= UCSSEL1 + UCSSEL0;

    // Ignore clockrate argument for now, just use clock source/2
    UCB0BR0 = 0x02;
    UCB0BR1 = 0x00;

    // Set up pins used by peripheral unit
    MCU_IO_PERIPHERAL(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
    MCU_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);

    // Release for operation
    UCB0CTL1 &= ~UCSWRST;
}

//----------------------------------------------------------------------------------
//  USI
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_USI

void halSpiInit(uint8 clockrate)
{
    // Keep peripheral in reset state
    USICTL0 = USISWRST;

    // 8-bit SPI Master 3-pin mode
    USICTL0 |= USIPE7 +  USIPE6 + USIPE5 + USIMST + USIOE;
    USICTL1  = USICKPH;

    // Use SMCLK as clock source, bit rate is clock source/2
    USICKCTL = USIDIV0 + USISSEL1;

    // Set up pins used by peripheral unit
    MCU_IO_PERIPHERAL(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
    MCU_IO_PERIPHERAL(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN);
    MCU_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);

    // Release for operation
    USICTL0 &= ~USISWRST;

    // Ensure SDO (SIMO) low
    USISRL = 0x00;
    USICNT = 1;
}

//----------------------------------------------------------------------------------
//  BITBANG
//----------------------------------------------------------------------------------
#elif defined HAL_SPI_INTERFACE_BITBANG

void halSpiInit(uint8 clockrate)
{
    // Set up pins
    MCU_IO_INPUT(HAL_SPI_SOMI_PORT, HAL_SPI_SOMI_PIN);
    MCU_IO_OUTPUT(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN, 0);
    MCU_IO_OUTPUT(HAL_SPI_CLK_PORT,  HAL_SPI_CLK_PIN, 0);
    MCU_IO_OUTPUT(HAL_SPI_CS_PORT, HAL_SPI_CS_PIN, 1);
}

uint8 halSpiBitbangOut(uint8 data)
{
    uint8 i;
    uint8 rc = 0;

    for(i = 8; i > 0; i--)
    {
        // clock low
        MCU_IO_CLR(HAL_SPI_CLK_PORT, HAL_SPI_CLK_PIN);

        if (data & 0x80)
            MCU_IO_SET(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
        else
            MCU_IO_CLR(HAL_SPI_SIMO_PORT, HAL_SPI_SIMO_PIN);
        data <<= 1;

        // clock high
        MCU_IO_SET(HAL_SPI_CLK_PORT, HAL_SPI_CLK_PIN);

        rc <<= 1;
        if (HAL_SPI_SOMI_VAL)
            rc |= 0x01;
    }

    // clock low
    MCU_IO_CLR(HAL_SPI_CLK_PORT, HAL_SPI_CLK_PIN);

    return(rc);
}

uint8 halSpiBitbangIn(void)
{
    uint8 i;
    uint8 data = 0;

    for(i = 8; i > 0; i--)
    {
        // clock high
        MCU_IO_SET(HAL_SPI_CLK_PORT, HAL_SPI_CLK_PIN);

        data <<= 1;
        if (HAL_SPI_SOMI_VAL)
            data |= 0x01;

        // clock low
        MCU_IO_CLR(HAL_SPI_CLK_PORT, HAL_SPI_CLK_PIN);
    }

    return(data);
}

//----------------------------------------------------------------------------------
//  Unknown interface
//----------------------------------------------------------------------------------
#else
#error "You must select an SPI interface (USART/USCI/USI/BITBANG)"
#endif
