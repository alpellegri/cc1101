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
#define RADIO_MODE_TX        1
#define RADIO_MODE_RX        2

#define RX_OK                0
#define RX_LENGTH_VIOLATION  1
#define RX_CRC_MISMATCH      2
#define RX_FIFO_OVERFLOW     3


//----------------------------------------------------------------------------------
//  Variables used in this file
//----------------------------------------------------------------------------------
static volatile uint8 radioMode;
static volatile uint8 radioModeSet = FALSE;
static volatile uint8 buttonPushed = FALSE;
static volatile uint8 packetSent;
static volatile uint8 packetReceived;

static uint8 data[64];

//----------------------------------------------------------------------------------
//  Function declarations
//----------------------------------------------------------------------------------
static uint8 txSendPacket(uint8* data, uint8 length);
static uint8 rxRecvPacket(uint8* data, uint8* length);
static void  txISR(void);
static void  rxISR(void);
static void  myTxButtonISR(void);
static void  myRxButtonISR(void);


//----------------------------------------------------------------------------------
//  void txISR(void)
//
//  DESCRIPTION:
//    This function is called (in interrupt context) every time a packet has been
//    transmitted.
//----------------------------------------------------------------------------------
static void txISR(void)
{
    packetSent = TRUE;
}

//----------------------------------------------------------------------------------
//  void rxISR(void)
//
//  DESCRIPTION:
//    This function is called (in interrupt context) every time a packet has been
//    revceived.
//----------------------------------------------------------------------------------
static void rxISR(void)
{
    packetReceived = TRUE;
}

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
    if (payloadLength > 61)
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
//    This is the main entry of the "link" application. It sets up the board and
//    lets the user select operating mode (RX or TX) by pressing either button
//    S1 or S2. In TX mode, one packet (with arbitrary length) is sent over the air
//    every time the button is pressed. In RX mode, the LCD will e updated every time
//    a new packet is successfully received. If an error occurred during reception,
//    one of the LEDs will flash.
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
    halRfWriteReg(CC2500_MCSM0,    0x18);   // Calibration from IDLE to TX/RX
    halRfWriteReg(CC2500_MCSM1,    0x00);   // No CCA, IDLE after TX and RX
    halRfWriteReg(CC2500_PKTCTRL0, 0x45);   // Enable data whitening and CRC
    halRfWriteReg(CC2500_PKTCTRL1, 0x04);   // Enable append mode
    halRfWriteReg(CC2500_IOCFG0,   0x06);   // Set GDO0 to be packet received signal


    // In this example, the packets being sent are smaller than the size of the
    // FIFO, thus all larger packets should be discarded. The packet length
    // filtering on the receiver side is necessary in order to handle the
    // CC2500 RX FIFO overflow errata, described in the CC2500 Errata Note.
    // Given a FIFO size of 64, the maximum packet is set such that the FIFO
    // has room for the length byte + payload + 2 appended status bytes (giving
    // a maximum payload size of 64 - 1 - 2 = 61.
    halRfWriteReg(CC2500_PKTLEN,     61);   // Max payload data length



    if (radioMode == RADIO_MODE_RX)
    {
        uint16 counter;
        uint8 payloadLength;

        // Connect RX interrupt to event on GDO0
        halDigioIntSetEdge(&pinGDO0, HAL_DIGIO_INT_FALLING_EDGE);
        halDigioIntConnect(&pinGDO0, &rxISR);
        halDigioIntEnable(&pinGDO0);

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
        uint8 packetLength;

        // Connect TX interrupt to event on GDO0
        halDigioIntSetEdge(&pinGDO0, HAL_DIGIO_INT_FALLING_EDGE);
        halDigioIntConnect(&pinGDO0, &txISR);
        halDigioIntEnable(&pinGDO0);

        counter = 0;
        while (TRUE)
        {
            // Create a dummy packet
            preparePacket(counter, data, &packetLength);

            // Send packet
            txSendPacket(data, packetLength);

            // Display number of packets sent
            halLcdWriteValue(++counter, HAL_LCD_RADIX_HEX, 0);

            // Wait for user to press button to continue
            key = halIntLock();
            while (!buttonPushed)
            {
                halMcuSetLowPowerMode(HAL_MCU_LPM_3);
                key = halIntLock();
            }
            halIntUnlock(key);
            buttonPushed = FALSE;
        }
    }
}


//----------------------------------------------------------------------------------
//  uint8 txSendPacket(uint8* data, uint8 length)
//
//  DESCRIPTION:
//    Send a packet that is smaller than the size of the FIFO, making it possible
//    to write the whole packet at once. Wait for the radio to signal that the packet
//    has been transmitted.
//
//  ARGUMENTS:
//    data   - Data to send. First byte contains length byte
//    length - Total length of packet to send
//
//  RETURNS:
//    This function always returns 0.
//----------------------------------------------------------------------------------
static uint8 txSendPacket(uint8* data, uint8 length)
{
    uint16 key;
    packetSent = FALSE;

    // Write data to FIFO
    halRfWriteFifo(data, length);

    // Set radio in transmit mode
    halRfStrobe(CC2500_STX);

    // Wait for packet to be sent
    key = halIntLock();
    while(!packetSent)
    {
        halMcuSetLowPowerMode(HAL_MCU_LPM_3);
        key = halIntLock();
    }
    halIntUnlock(key);
    return(0);
}

//----------------------------------------------------------------------------------
//  uint8 rxRecvPacket(uint8* data, uint8* length)
//
//  DESCRIPTION:
//    Receive a packet that is smaller than the size of the FIFO, i.e. wait for the
//    complete packet to be received before reading from the FIFO. This function sets
//    the CC1100/CC2500 in RX and waits for the chip to signal that a packet is received.
//
//  ARGUMENTS:
//    data   - Where to write incoming data.
//    length - Length of payload.
//
//  RETURNS:
//    0 if a packet was received successfully.
//    1 if chip is in overflow state (packet longer than FIFO).
//    2 if the length of the packet is illegal (0 or > 61).
//    3 if the CRC of the packet is not OK.
//----------------------------------------------------------------------------------
static uint8 rxRecvPacket(uint8* data, uint8* length)
{
    uint8 packet_status[2];
    uint8 status;
    uint16 key;

    packetReceived = FALSE;
    status = RX_OK;

    // Set radio in RX mode
    halRfStrobe(CC2500_SRX);

    // Wait for incoming packet
    key = halIntLock();
    while(!packetReceived)
    {
        halMcuSetLowPowerMode(HAL_MCU_LPM_3);
        key = halIntLock();
    }
    halIntUnlock(key);

    // Read first element of packet from the RX FIFO
    status = halRfReadFifo(length, 1);

    if ((status & CC2500_STATUS_STATE_BM) == CC2500_STATE_RX_OVERFLOW)
    {
        halRfStrobe(CC2500_SIDLE);
        halRfStrobe(CC2500_SFRX);
        status = RX_FIFO_OVERFLOW;
    }
    else if (*length == 0 || *length > 61)
    {
        halRfStrobe(CC2500_SIDLE);
        halRfStrobe(CC2500_SFRX);
        status = RX_LENGTH_VIOLATION;
    }
    else
    {
        // Get payload
        halRfReadFifo(data, *length);

        // Get the packet status bytes [RSSI, LQI]
        halRfReadFifo(packet_status, 2);

        // Check CRC
        if ((packet_status[1] & CC2500_LQI_CRC_OK_BM) != CC2500_LQI_CRC_OK_BM)
        {
            status = RX_CRC_MISMATCH;
        }
        else
        {
            status = RX_OK;
        }
    }
    return(status);
}
