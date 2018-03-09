/******************************************************************************
    Filename: hal_lcd.c

    Description:

      This file implements support for the Softbaugh SBLCDA4 LCD Display.
      For further details, see
          http://www.softbaugh.com/downloads/SBLCDA4_Specification.pdf

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include "hal_types.h"
#include "hal_lcd.h"
#include "hal_board.h"


// Temporary defines
#define a 0x01
#define b 0x02
#define c 0x04
#define d 0x08
#define f 0x10
#define g 0x20
#define e 0x40
#define h 0x80

static const uint8 lcd_chars[] =
{
    a+b+c+d+e+f,          //  0 Displays "0"
    b+c,                  //  1 Displays "1"
    a+b+d+e+g,            //  2 Displays "2"
    a+b+c+d+g,            //  3 Displays "3"
    b+c+f+g,              //  4 Displays "4"
    a+c+d+f+g,            //  5 Displays "5"
    a+c+d+e+f+g,          //  6 Displays "6"
    a+b+c,                //  7 Displays "7"
    a+b+c+d+e+f+g,        //  8 Displays "8"
    a+b+c+d+f+g,          //  9 Displays "9"
    a+b+c+f+g+e,          // 10 Displays "A"
    f+e+g+c+d,            // 11 Displays "B"
    a+f+e+d,              // 12 Displays "C"
    b+c+d+e+g,            // 13 Displays "D"
    a+f+g+e+d,            // 14 Displays "E"
    a+f+g+e               // 15 Displays "F"
};

// Undefine temporary defines
#undef a
#undef b
#undef c
#undef d
#undef e
#undef f
#undef g
#undef h



//----------------------------------------------------------------------------------
//  void halLcdInit(void)
//
//  DESCRIPTION:
//  Initialize and enable the Softbaugh SBLCDA4 LCD used on the Experimenter Board.
//----------------------------------------------------------------------------------
void halLcdInit(void)
{
    // Set up pins used by peripheral unit
    P5SEL |= BIT2 + BIT3 + BIT4;

    halLcdClear();
    LCDACTL = LCDFREQ_64 + LCD4MUX + LCDON;   // 4mux LCD, segs0-23 enabled
    LCDAPCTL0 = LCDS4 + LCDS8 + LCDS12 + LCDS16 + LCDS20 + LCDS24;
    LCDAPCTL1 = 0;
    LCDAVCTL0 = LCDCPEN;
    LCDAVCTL1 = VLCD_4;
}

//----------------------------------------------------------------------------------
//  void halLcdClear(void)
//
//  DESCRIPTION:
//  Clear display.
//----------------------------------------------------------------------------------
void halLcdClear(void)
{
    uint8 i;
    for(i = 0; i < 20; i++)
    {
        LCDMEM[i] = 0;
    }
}

//----------------------------------------------------------------------------------
//  void halLcdWriteSymbol(uint8 symbol, bool on)
//
//  DESCRIPTION:
//  Display an LCD specific symbol.
//
//  ARGUMENTS:
//
//    symbol - the symbol to display
//    on     - set to TRUE ( != 0 ) to turn symbol on, FALSE (0) to turn symbol off
//
//----------------------------------------------------------------------------------
void halLcdWriteSymbol(uint8 symbol, uint8 on)
{
    uint8 segment;
    uint8 bitmask;

    switch (symbol)
    {
        case HAL_LCD_SYMBOL_ANT:
            segment = 11;
            bitmask = 0x10;
            break;
        case HAL_LCD_SYMBOL_RX:
            segment = 12;
            bitmask = 0x04;
            break;
        case HAL_LCD_SYMBOL_TX:
            segment = 12;
            bitmask = 0x02;
            break;
        default:
            return;
    }

    if (on)
        LCDMEM[segment] |= bitmask;
    else // off
        LCDMEM[segment] &= ~bitmask;
}

//----------------------------------------------------------------------------------
//  uint32 itobcd(uint16 value)
//
//  DESCRIPTION
//  Convert a value to a binary coded decimal (BCD) number.
//  Note that a 16 bit value will need 20 bits for its BCD counterpart,
//  and a 32 bit value would need 40 bits. Thus, we restrict ourselves to
//  a 16 bit value.
//
//  ARGUMENTS:
//
//    value - the value to convert
//
//  RETURNS:
//  Returns the value as a binary coded decimal
//----------------------------------------------------------------------------------
static uint32 itobcd(uint16 value)
{
    uint32 bcd = 0;
    uint32 val = (uint32)value;
    uint8  j = 0;
    while (val > 9)
    {
        bcd |= ((val % 10) << j);
        val /= 10;
        j += 4;
    }
    return (bcd | (val << j));
}


//----------------------------------------------------------------------------------
//  void halLcdWriteValue(uint16 value, uint8 radix, uint8 line)
//
//  DESCRIPTION:
//  Write a 16 bit value, either in hexadecimal or decimal, on the specified
//  line on the LCD.
//
//  ARGUMENTS:
//
//    value - the value to display
//    radix - one of HAL_LCD_RADIX_DEC or HAL_LCD_RADIX_HEX
//    line  - ignored, as the SBLCDA4 only has one line
//
//----------------------------------------------------------------------------------
void halLcdWriteValue(uint16 value, uint8 radix, uint8 line)
{
    uint32 bcd;
    switch (radix)
    {
        case HAL_LCD_RADIX_HEX:

            LCDMEM[2] = lcd_chars[value & 0x000f];
            value >>= 4;
            LCDMEM[3] = lcd_chars[value & 0x000f];
            value >>= 4;
            LCDMEM[4] = lcd_chars[value & 0x000f];
            value >>= 4;
            LCDMEM[5] = lcd_chars[value & 0x000f];
            break;

         case HAL_LCD_RADIX_DEC:

            bcd = itobcd(value);
            LCDMEM[2] = lcd_chars[bcd & 0x000f];
            bcd >>= 4;
            LCDMEM[3] = lcd_chars[bcd & 0x000f];
            bcd >>= 4;
            LCDMEM[4] = lcd_chars[bcd & 0x000f];
            bcd >>= 4;
            LCDMEM[5] = lcd_chars[bcd & 0x000f];
            bcd >>= 4;
            LCDMEM[6] = lcd_chars[bcd & 0x000f];
            break;

         default:
            break;
    }
}

