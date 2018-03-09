/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function transmits a packet                                           *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfSendPacketSerial.c                                                       *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:        See end of file                                                            *
 ******************************************************************************************************/
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\culsrf04.h>




//-------------------------------------------------------------------------------------------------------
// Defines
#define GDO1_PIN        P0_1
#define SERIAL_CLK      GDO1_PIN

#define GDO0_PIN        P0_6
#define SERIAL_DATA     GDO0_PIN
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacketSerial(BYTE *txBuffer, UINT8 size, UINT8 startOfPayload, BOOL crcEnable)
//
//  DESCRIPTION:
//      This function transmits a packet using synchronous serial mode. The radio must be configured to 
//      use GIO1 as the serial clock (IOCFG1 = 0x0B) and GDO0 as serial synchronous data output
//      (IOCFG0 = 0x0C).
//      The packet should have the following format:
//      
//      ----------------------------------------------------------------------------------
//      | preamble | sync3 | sync2 | sync1 | sync0 | length byte | payload | crc1 | crc0 |
//      ----------------------------------------------------------------------------------
//
//      The length byte and the crc is optional.
//      Be aware that modifications to this function might be necessary if it is ported to other
//      MCUs or if the data rate is increased, due to timing issues.
//      The function is only meant to give an indication on how to use the serial synchronous mode
//      of the CCxx00.    
//      
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to array containing the packet to be sent.
//          For example:
//          BYTE xdata txBuffer[] = {0x55,  0x55,   0x55,   0x55,  Preamble
//                                   SYNC3, SYNC2,  SYNC1,  SYNC0, Sync word (it is not necessary to
//                                                                 transmit 4 sync bytes but the culSyncSearch
//                                                                 function in the cul library must be  
//                                                                 modified if other sync lengths should be used. 
//                                   3,     'A',    'B',    'C',   length byte + payload (length byte is optional, 
//                                                                 depending on how the receiver is configured
//                                                                 (fixed or variable packet length)
//                                   CRC1, CRC0};                  CRC bytes (optional)
//                                                                 This is just 2 dummy bytes which will be 
//                                                                 replaced with 'real' crc bytes if crc calculation 
//                                                                 is enabled
//
//      UINT8 size
//          Size of the txBuffer
// 
//      UINT8 startOfPayload
//          Indicates where in the txBuffer the crc calculation should start (crc is calculated over the
//          optional length byte and the payload.
//
//      BOOL crcEnable
//          TRUE: Calculate CRC and replace CRC1 and CRC0 in txBuffer with 'real' crc bytes
//          FALSE: CRC is not calculated and TX mode is terminated after the payload has been sent  
//-------------------------------------------------------------------------------------------------------
void halRfSendPacketSerial(BYTE *txBuffer, UINT8 size, UINT8 startOfPayload, BOOL crcEnable) {

    UINT8 data i;
    UINT16 xdata checksum;
    UINT8 data dataByte = 0;
    UINT8 data lengthByte;
    UINT8 data bufferIndex = 0;

    if (crcEnable) {

        // Generate CRC
        checksum = CRC_INIT;   //Init value for CRC calculation

        for(i = startOfPayload; i < (size - 2); i++)
            checksum = culCalcCRC(txBuffer[i], checksum);



        txBuffer[size - 2] = checksum >> 8;     // CRC1
        txBuffer[size - 1] = checksum & 0x00FF; // CRC0
    }

    lengthByte = txBuffer[0];
                    
    // MSB of the first byte needs to ready on the data line before strobing TX
    if (((lengthByte & MSB) >> 7) == 0)
        SERIAL_DATA = 0;
    else
        SERIAL_DATA = 1;

    lengthByte = lengthByte << 1;

    // Enter TX mode
    halSpiStrobe(CCxxx0_STX);

    // Transmit the first byte
    for (i = 1; i < 8; i++) {
        while (!SERIAL_CLK);
                     
        if (((lengthByte & MSB) >> 7) == 0)
            SERIAL_DATA = 0;
        else
            SERIAL_DATA = 1;
        
        lengthByte = lengthByte << 1;

        while (SERIAL_CLK);
    }
    
    // Tranmit the rest of the packet                               
    for (bufferIndex = 1; bufferIndex < size; bufferIndex++) {
        dataByte = txBuffer[bufferIndex];
        for (i = 0; i < 8; i++) {
            while (!SERIAL_CLK);
            
            if (((dataByte & MSB) >> 7) == 0)
                SERIAL_DATA = 0;
            else
                SERIAL_DATA = 1;

            dataByte = dataByte << 1;
            while (SERIAL_CLK);
        }
    }
    while (!SERIAL_CLK);
    while (SERIAL_CLK);  
    
    // Transmit 13 dummy bits (must be 26 if Manchester mode is enabled) 
    SERIAL_DATA = 0;  
    for (i = 0; i < 13; i++) {
        while (!SERIAL_CLK);
        while (SERIAL_CLK);
    }            

    // Enter IDLE mode
    halSpiStrobe(CCxxx0_SIDLE);
}// halRfSendPacketSerial




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: RfSendPacketSerial.c,v $
 * Revision 1.2  2007/06/22 13:31:42  a0190596
 * Changed number of dummy bytes (see errata note)
 *
 * Revision 1.1  2005/11/09 09:40:17  sna
 * Initial version in CVS.
 *
 *
 *
 ******************************************************************************************************/