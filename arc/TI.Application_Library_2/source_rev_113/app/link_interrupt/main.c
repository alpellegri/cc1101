/******************************************************************************
    Filename: main.c

    Copyright 2007 Texas Instruments, Inc.
******************************************************************************/

#include <hal_types.h>
#include <hal_defs.h>
#include <hal_board.h>
#include <hal_uart.h>
#include <hal_lcd.h>
#include <hal_led.h>
#include <hal_spi.h>
#include <hal_int.h>
#include <hal_mcu.h>
#include <hal_rf.h>
#include <cc2500.h>
#include <stdlib.h>

#include "my_rf_settings.h"


//----------------------------------------------------------------------------------
//  Constants used in this file
//----------------------------------------------------------------------------------
#define RADIO_MODE_TX  1
#define RADIO_MODE_RX  2


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
static volatile uint8 radioMode;
static volatile uint8 radioModeSet = FALSE;
static volatile uint8 buttonPushed = FALSE;
static uint8 data[256];


//----------------------------------------------------------------------------------
//  Function declarations
//----------------------------------------------------------------------------------
extern void  rxInit(void);
extern void  txInit(void);
extern uint8 txSendPacket(uint8* data, uint8 length);
extern uint8 rxRecvPacket(uint8* data, uint8* length);



//----------------------------------------------------------------------------------
//  void myRxButtonISR(void)
//
//  DESCRIPTION:
//    This function is called when the S1 button is pressed.
//    Selects mode of operation the first time it runs.
//----------------------------------------------------------------------------------
static void myRxButtonISR(void)
{
    if (!radioModeSet)
    {
        radioMode = RADIO_MODE_RX;
        radioModeSet = TRUE;
        halLcdWriteSymbol(HAL_LCD_SYMBOL_ANT, TRUE);
        halLcdWriteSymbol(HAL_LCD_SYMBOL_RX, TRUE);
    }
    buttonPushed = TRUE;
}

//----------------------------------------------------------------------------------
//  void myTxButtonISR(void)
//
//  DESCRIPTION:
//    This function is called when the S2 button is pressed.
//    Selects mode of operation the first time it runs.
//----------------------------------------------------------------------------------
static void myTxButtonISR(void)
{
    if (!radioModeSet)
    {
        radioMode = RADIO_MODE_TX;
        radioModeSet = TRUE;
        halLcdWriteSymbol(HAL_LCD_SYMBOL_ANT, TRUE);
        halLcdWriteSymbol(HAL_LCD_SYMBOL_TX, TRUE);
    }
    buttonPushed = TRUE;
}

//----------------------------------------------------------------------------------
//  void preparePacket(uint8 id, uint8* data, uint8* length)
//
//  DESCRIPTION:
//    Set up a dummy packet, where the first byte contains the length of the payload
//    and the first byte of the payload contains a packet id.
//----------------------------------------------------------------------------------
static void preparePacket(uint8 id, uint8* data, uint8* length)
{
    uint8 i;
    static uint8 payloadLength;

    payloadLength++;
    if (payloadLength > 254)
        payloadLength = 1;

    // First byte of packet contains the length of the payload
    data[0] = payloadLength;

    // First byte of payload contains an id
    data[1] = id;

    // Fill rest of packet with dummy data
    for (i = 2; i <= payloadLength; i++)
        data[i] = i;

    // Packet length is payload + length byte
    *length = payloadLength + 1;
}


//----------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//    This is the main entry of the "link_interrupt" application. It sets up the board
//    and lets the user select operating mode (RX or TX) by pressing either button
//    S1 or S2. In TX mode, a packet containing random data (with arbitrary length)
//    is written to the FIFO of the RF chip continously. In RX mode, the LCD will be
//    updated every time a new packet is successfully received. If an error occurs
//    during reception, a LED will flash.
//----------------------------------------------------------------------------------
void main (void)
{
    uint8  id;
    uint8  ver;
    uint16 key;

    halBoardInit();

    halUartWrite("CC1100 & CC2500 Link Example\r\n", 30);

    halRfResetChip();

    id  = halRfGetChipId();
    ver = halRfGetChipVer();
    halLcdWriteValue((id << 8) | ver, HAL_LCD_RADIX_HEX, 0);

    // Put radio to sleep
    halRfStrobe(CC2500_SPWD);


    // Set up interrupts for events on the S1 and S2 buttons
    halDigioIntSetEdge(&pinS1, HAL_DIGIO_INT_RISING_EDGE);
    halDigioIntConnect(&pinS1, &myRxButtonISR);
    halDigioIntEnable(&pinS1);

    halDigioIntSetEdge(&pinS2, HAL_DIGIO_INT_RISING_EDGE);
    halDigioIntConnect(&pinS2, &myTxButtonISR);
    halDigioIntEnable(&pinS2);

    // Wait for user to select operating mode
    key = halIntLock();
    while (!buttonPushed)
    {
        halMcuSetLowPowerMode(HAL_MCU_LPM_3);
        key = halIntLock();
    }
    halIntUnlock(key);
    buttonPushed = FALSE;

    // Setup chip with register settings from SmartRF Studio
    halRfConfig(&myRfConfig, myPaTable, myPaTableLen);

    // Additional chip configuration for this example
    halRfWriteReg(CC2500_MCSM0,    0x18); // Calibration from IDLE to TX/RX
    halRfWriteReg(CC2500_MCSM1,    0x00); // No CCA, IDLE after TX and RX
    halRfWriteReg(CC2500_PKTCTRL0, 0x45); // Enable data whitening and CRC
    halRfWriteReg(CC2500_PKTCTRL1, 0x04); // Enable append mode


    if (radioMode == RADIO_MODE_RX)
    {
        uint16 counter;
        uint8  payloadLength;

        rxInit();
        counter = 0;
        while (TRUE)
        {
            if (rxRecvPacket(data, &payloadLength) == 0)
            {
                halLcdWriteValue(++counter, HAL_LCD_RADIX_HEX, 0);
            }
            else
            {
                halLedToggle(1);
                halMcuWaitUs(20000);
                halLedToggle(1);
            }
        }
    }
    else if (radioMode == RADIO_MODE_TX)
    {
        uint16 counter;
        uint8  packetLength;

        txInit();
        counter = 0;
        while (TRUE)
        {
            // Create a dummy packet
            preparePacket(counter, data, &packetLength);

            // Send packet
            txSendPacket(data, packetLength);

            // Display number of packets sent
            halLcdWriteValue(++counter, HAL_LCD_RADIX_HEX, 0);

            // Wait a bit such that the receiver is not saturated
            halMcuWaitUs(2000);
        }
    }
}
