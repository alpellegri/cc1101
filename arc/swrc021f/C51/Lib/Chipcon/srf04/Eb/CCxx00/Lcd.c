/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file implements LCD display initialization                            *
 *      ***   +             and functions for updating the LCD display                                 *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Lcd.c                                                                      *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon, CCxx00                                                            *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:        See end of file                                                            *
 ******************************************************************************************************/
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>
#include <intrins.h>




//-------------------------------------------------------------------------------------------------------
//  This file contains all necessary defines, macroes and function for writing to the LCD display.
//  The i2c communication is done using SMBus interrupt and timer 0 is used to generate SCL.
//  Below is an example on how to write a text to the display. As, showed below, only two of the 
//  functions in this file, (ebLcdInit and ebLcdUpdate) is needed to be called from the user application.
//  The other functions are only used internally in this file.
//
//      void ebLcdInit(void)
//      BYTE ebLcdUpdate(UINT8 *pLine1, UINT8 *pLine2)
//      void SMBusIsr(void) interrupt INUM_SMB0
//      BYTE ebLcdConvertChar(BYTE c)
//      void ebI2CHandler(void)
//
//  ebLcdInit();
//  ebLcdUpdate("Connecting", "      Smarter");
//
//  |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//  | C | o | n | n | e | c | t | i | n | g |   |   |   |   |   |   |
//  |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---| 
//  |   |   |   |   |   |   | S | m | a | r | t | e | r |   |   |   |
//  |---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Status vector (SMBus) - top 4 bits only
#define SMB_MTSTA   0xE0    // (MT) start transmitted
#define SMB_MTDB    0xC0    // (MT) data byte transmitted, ACK received
#define SMB_MRDB    0x80    // (MR) data byte received

// Defines used by the ebLcdInit and ebLcdUpdate functions 
#define LINE_SIZE  16
#define LINE1      0x80
#define LINE2      0xC0
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// LCD slave address
#define LCD_ADDR        0x76

// Defines for Register Select (RS). RS selects the register to be accessed for read and write
#define RS_0            0x00        // RS = 0 => selects instruction register for write and busy flag
                                    // and address counter for read
#define RS_1            0x40        // RS = 1 => selects the data register for both read and write

// Reset pin
#define RESET_N         P2_3

#define SPACE           0x9B
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Struct used by the ebLcdUpdate
typedef struct {
    BOOL doTransfer;
    BOOL transferInProgress;
    UINT8 bytesLeft;
    UINT8 position;
} I2C_DATA;
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Variables used by ebLcdInit, ebLcdUpdate, and the SMBus ISR 
BYTE i2cBuffer[47];       // It takes 47 bytes to update the entire LCD display
I2C_DATA i2cData;
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  DESCRIPTION:
//      Converts ASCII characters to the character set the LCD display uses 
//-------------------------------------------------------------------------------------------------------
BYTE ebLcdConvertChar(BYTE c);




//-------------------------------------------------------------------------------------------------------
//  DESCRIPTION: 
//      This function is responsible for all traffic on the I2C interface.
//-------------------------------------------------------------------------------------------------------
void ebI2CHandler(void);




//-------------------------------------------------------------------------------------------------------
//  void ebLcdInit(void)
//
//  DESCRIPTION:
//      Function used to initialize the LCD display. The SMBus uses timer 0 to generate SCL
//      This function also writes 32 SPACE characters to the display to clear it.
//-------------------------------------------------------------------------------------------------------
void ebLcdInit(void) {

    UINT8 i;

    // Setup SCL High and Low times using timer 0 (THighMin = TLowMin = 1 / fClockSourceOverflow)
    TH0 = 150;  // SCL low time = (256 - 150) * (1 / 24000000) = 4,42 us 
    TL0 = TH0;
  
    // Set mode
    TMOD &= 0xFC;
    TMOD |= MODE_2;     // 8-bit counter/timer with auto-reload
   
    CKCON |= BM_T0M;    // Counter/Timer 0 uses the system clock
    
    TIMER0_RUN(TRUE);   // Start timer 0    

    // Initialize SMBus 
    SMB0CF |= BM_ENSMB;

     // Enable SMBus interrupt
    INT_ENABLE(INUM_SMB0, INT_ON);
    ENABLE_GLOBAL_INT(TRUE);

    // Initializes the I2C handler's data structure
    i2cData.doTransfer = FALSE;
    i2cData.transferInProgress = FALSE;
    i2cData.position = 0;

    // Reset the display
    RESET_N = 0;
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    _nop_();
    RESET_N = 1;
 
    i2cBuffer[0] = 7;
    i2cBuffer[1] = LCD_ADDR;
    i2cBuffer[2] = RS_0;            // Instruction Register
    i2cBuffer[3] = 0x0C;            // Display control          D = 1:      Display On
                                    //                          C = 0:      Cursor Off
                                    //                          B = 0:      Cursor character blink off
    i2cBuffer[4] = 0x21;            // Function set             H = 1:      Use extended instruction set
    i2cBuffer[5] = 0xA0;            // Set DDRAM address        ADD = 0x20
    i2cBuffer[6] = 0x07;            // Display configuration    P = 1:      Column data right to left
                                    //                          Q = 1:      Row data, bottom to top
    i2cBuffer[7] = 0x24;            // Function set             DL = 0:     4 bits
                                    //                          M = 1:      2-line by 16 display
                                    //                          SL = 0:     MUX1:18
                                    //                          H = 0:      Use basic instruction set
    i2cBuffer[8] = 0;
    i2cData.doTransfer = TRUE;
    ebI2CHandler();
    while (i2cData.doTransfer == TRUE);     // Wait for tranfer to finish
    
    // Clear display by writing SPACE
    i2cBuffer[0] = 3;
    i2cBuffer[1] = LCD_ADDR;
    i2cBuffer[2] = RS_0;
    i2cBuffer[3] = LINE1;

    i2cBuffer[4] = 18;
    i2cBuffer[5] = LCD_ADDR;
    i2cBuffer[6] = RS_1;

    for (i = 0; i < (LINE_SIZE); i++)
        i2cBuffer[i + 7] = SPACE;

    i2cBuffer[23] = 3;
    i2cBuffer[24] = LCD_ADDR;
    i2cBuffer[25] = RS_0;
    i2cBuffer[26] = LINE2;

    i2cBuffer[27] = 18;
    i2cBuffer[28] = LCD_ADDR;
    i2cBuffer[29] = RS_1;

    for (i = 0; i < (LINE_SIZE); i++)
        i2cBuffer[i + 30] = SPACE;

    i2cBuffer[46] = 0;


    i2cData.doTransfer = TRUE;
    ebI2CHandler();
    while (i2cData.doTransfer == TRUE);     // Wait for tranfer to finish
}// ebLcdInit




//-------------------------------------------------------------------------------------------------------
//  BYTE ebLcdUpdate(UINT8 *pLine1, UINT8 *pLine2)
//
//  DESCRIPTION:
//      This function converts the two text strings from ASCII to the character set used by the LCD 
//      display. It also sets up the i2cBuffer corretly with slave address, register select, etc, 
//      for accessing both lines of the display.
//      A complete LCD display update is done in four I2C write operations:
//          1. Move to the first character on the first line
//          2. Write 16 characters to the display
//          3. Move to the first character on the second line
//          4. Write another 16 characters to the display
//
//  ARGUMENTS:
//      UINT8 *pLine1
//          Pointer to text string to be written to line 1
//      UINT8 *pLine2
//          Pointer to text string to be written to line 2
//
//  RETURN VALUE:
//      BYTE
//          FALSE:  A transfer is already in progress and nothing is done
//          TRUE:   A new transfer is started    
//-------------------------------------------------------------------------------------------------------
BYTE ebLcdUpdate(UINT8 *pLine1, UINT8 *pLine2) {
    UINT8 i = 0;
    UINT8 *pTemp;

    if (!i2cData.doTransfer) {
        i2cBuffer[0] = 3;
        i2cBuffer[1] = LCD_ADDR;
        i2cBuffer[2] = RS_0;

        i2cBuffer[4] = 18;
        i2cBuffer[5] = LCD_ADDR;
        i2cBuffer[6] = RS_1;
    
        if (((pLine1[0] == NULL) && (pLine2[0] != NULL)) || ((pLine2[0] == NULL) && (pLine1[0] != NULL))) {
            
            // which line ?
            if (pLine1[0]) {
                i2cBuffer[3] = LINE1;
                pTemp = pLine1;
            } else {
                i2cBuffer[3] = LINE2;
                pTemp = pLine2;
            }

            do {
                i2cBuffer[i + 7] = ebLcdConvertChar(pTemp[i]);
            } while ((pTemp[i++] != NULL) && (i < LINE_SIZE));

            if (i < LINE_SIZE) {
                for ( i = i + 7; i < 23; i++)
                    i2cBuffer[i] = SPACE;
            }
            i2cBuffer[23] = 0;
        } 
        else if ((pLine1[0] != NULL) && (pLine2[0] != NULL)) {
            
            i2cBuffer[3] = LINE1;
            i2cBuffer[23] = 3;
            i2cBuffer[24] = LCD_ADDR;
            i2cBuffer[25] = RS_0;
            i2cBuffer[26] = LINE2;

            i2cBuffer[27] = 18;
            i2cBuffer[28] = LCD_ADDR;
            i2cBuffer[29] = RS_1;

            do {
                i2cBuffer[i + 7] = ebLcdConvertChar(pLine1[i]);
            } while ((pLine1[i++] != NULL) && (i < LINE_SIZE));

            if (i < LINE_SIZE) {
                for ( i = i + 7; i < 23; i++)
                    i2cBuffer[i] = SPACE;
            }

            i = 0;
            do {
                i2cBuffer[i + 30] = ebLcdConvertChar(pLine2[i]);
            } while ((pLine2[i++] != NULL) && (i < LINE_SIZE));

            if (i < LINE_SIZE) {
                for ( i = i + 30; i < 46; i++)
                    i2cBuffer[i] = SPACE;
            }
            i2cBuffer[46] = 0;

        }
        i2cData.doTransfer = TRUE;
        ebI2CHandler();
        return TRUE;
    }
    return FALSE;       // A transfer is already in progress
}// ebLcdUpdate




//-------------------------------------------------------------------------------------------------------
//  DESCRIPTION:
//      SMBus Interrupt Service Routine (ISR)
//-------------------------------------------------------------------------------------------------------
void SMBusIsr(void) interrupt INUM_SMB0 {
    switch (SMB0CN & 0xF0) {

    // Address + R/W byte
    case SMB_MTSTA:
        SMB0DAT = i2cBuffer[i2cData.position++];
        STA = 0;
        break;

    // Data byte transmitted
    case SMB_MTDB:
        if (--i2cData.bytesLeft)
            SMB0DAT = i2cBuffer[i2cData.position++];
        else {
            STO = 1;
            i2cData.transferInProgress = FALSE;
            ebI2CHandler();
        }
        break;
    }
    SI = 0;
}// SMBusIsr




//-------------------------------------------------------------------------------------------------------
//  BYTE ebLcdConvertChar(BYTE c)
//
//  DESCRIPTION:
//      Converts ASCII characters to the character set the LCD display uses (works for all digits and
//      letters and some other common characters)
//-------------------------------------------------------------------------------------------------------
BYTE ebLcdConvertChar(BYTE c) {
    if (c == '$')
        c = 0x82;
    else if (c == '@')
        c = 0x80;
    else if (c == NULL)
        c = 0x9B;
    else if (((c > 0x1F) && (c < 0x24)) || ((c > 0x24) && (c < 0x40)) || ((c > 0x40) && (c < 0x5B)))
        c += 0x80;
    return c;
}// ebLcdConvertChar




//-------------------------------------------------------------------------------------------------------
//  void ebI2CHandler(void) 
//
//  DESCRIPTION: 
//      This function is responsible for all traffic on the I2C interface. The handler is in principle 
//      similar to that for the LCD, however, the transfer is time-consuming, and is done through the 
//      SMBus interrupt (see the SmbusIsr).
//
//      typedef struct {
//          BOOL doTransfer;
//          BOOL transferInProgress;
//          UINT8 bytesLeft;
//          UINT8 position;
//      } I2C_DATA;
//
//      The doTransfer flag is used to start the I2C handler, and will also indicate when the handler 
//      has finished. The transferInProgress flag is used to implement handshaking between the 
//      i2cHandler() function and the SMBus interrupt service routine. The position counter is an index 
//      into the i2cBuffer[] array. For each small transaction the I2C handler will initialize the 
//      number of bytes to transfer (copied from i2cBuffer[] into bytesLeft).
//-------------------------------------------------------------------------------------------------------
void ebI2CHandler(void) {
    if (i2cData.doTransfer) {
        if (!i2cData.transferInProgress) {
            i2cData.bytesLeft = i2cBuffer[i2cData.position++];

            if (i2cData.bytesLeft != 0) {
                i2cData.transferInProgress = TRUE;
                STA = 1;
            } else {
                i2cData.doTransfer = FALSE;
                i2cData.position = 0;
            }
        }
    }
}// ebI2CHandler




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: Lcd.c,v $
 * Revision 1.4  2005/11/09 09:35:19  sna
 * Increased the delay between setting RESET_N low and RESET_N high
 *
 * Revision 1.3  2005/10/25 09:49:44  sna
 * Corrected inconsistent operator syntax on conditional expressions.
 *
 * Revision 1.2  2005/02/07 14:43:48  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/
