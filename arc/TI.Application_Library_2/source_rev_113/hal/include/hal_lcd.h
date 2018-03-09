/***********************************************************************************
    Filename: hal_lcd.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_LCD_H
#define HAL_LCD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <hal_types.h>

enum {
    HAL_LCD_SYMBOL_ANT,
    HAL_LCD_SYMBOL_RX,
    HAL_LCD_SYMBOL_TX,
    HAL_LCD_SYMBOL_A0,
    HAL_LCD_SYMBOL_A1,
    HAL_LCD_SYMBOL_A2
};

enum {
    HAL_LCD_RADIX_DEC,
    HAL_LCD_RADIX_HEX
};

enum {
    HAL_LCD_LINE_1,
    HAL_LCD_LINE_2
};

//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void halLcdInit(void);
void halLcdClear(void);
void halLcdWriteValue(uint16 value, uint8 radix, uint8 line);
void halLcdWriteSymbol(uint8 symbol, uint8 on);
void halLcdWriteScreen(const char* line1, const char* line2);

#ifdef  __cplusplus
}
#endif

/**********************************************************************************/
#endif
