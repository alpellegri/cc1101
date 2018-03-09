/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function receives a packet using synchronous serial mode.             *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RfReceivePacketSerial.c                                                    *
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
//  BOOL halRfReceivePacketSerial
//  (BYTE *rxBuffer, UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0, UINT8 fixedLength, BOOL crcEnable)
//
//  DESCRIPTION:
//      This function receives a packet using synchronous serial mode. The radio must be configured to 
//      use GIO1 as the serial clock (IOCFG1 = 0x0B) and GDO0 as serial synchronous data output
//      (IOCFG0 = 0x0C).
//      The packets received by this function should have the following format:
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
//      BYTE *rxBuffer
//          Pointer to array where the received packet is being stored
//
//      UINT8 sync3, UINT8 sync2 UINT8 sync1, UINT8 sync0
//          Sync word to search for. If one wants to send a sync word of a different length it is 
//          necessary to modify the culSyncSearch function, found in the cul folder 
//          (cul = Chipcon Utility Library).
//
//      UINT8 fixedLength
//          Length of data to be received after sync word is found. If fixedLength = 0, this means that
//          the length byte is given as the first byte received after sync word is found.
//
//      BOOL crcEnable
//          If crcEnable is TRUE, two bytes are received after the payload. This is crc1 and crc0.
//          A crc check will be performed on the received data and compared to crc1 and crc0.
// 
//  RETURN VALUE:
//      BOOL
//          crcEnable == TRUE =>    TRUE:   CRC OK
//                                  FALSE:  CRC NOT OK
//          crcEnable == FALSE =>   Return TRUE when a packet is received            
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacketSerial
(BYTE *rxBuffer, UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0, UINT8 fixedLength, BOOL crcEnable) {

    UINT8 data i, j;
    UINT16 xdata checksum;
    UINT8 data dataByte = 0;
    UINT8 data lengthByte;
    UINT8 data bufferIndex;
    UINT8 data crcBytes;

    if (crcEnable)
        crcBytes = 2;
    else
        crcBytes = 0;

    halSpiStrobe(CCxxx0_SRX);

    // Wait until sync word is received
    culSyncSearch(sync3, sync2, sync1, sync0);

    if (fixedLength) {
        lengthByte = fixedLength;
        bufferIndex = 0;

        for (j = 0; j < (lengthByte + crcBytes); j++) {
        
            for (i = 0; i < 8; i++) {
                            
                while (!SERIAL_CLK);

                dataByte = dataByte << 1;

                if(SERIAL_DATA == 0)
                    dataByte &= 0xFE;
                else
                    dataByte |= 0x01;

                while (SERIAL_CLK);
            }
            rxBuffer[bufferIndex++] = dataByte;
        }
    } else {

        bufferIndex = 1;
    
        // Read length byte
        for (i = 0; i < 8; i++) {
                        
            while (!SERIAL_CLK);

            lengthByte = lengthByte << 1;

            if(SERIAL_DATA == 0)
                lengthByte &= 0xFE;
            else
                lengthByte |= 0x01;

            while (SERIAL_CLK);
        }
            
        // Read the rest of the payload + optionally the 2 CRC bytes
        for (j = 0; j < (lengthByte + crcBytes); j++) {
        
            for (i = 0; i < 8; i++) {
                            
                while (!SERIAL_CLK);

                dataByte = dataByte << 1;

                if(SERIAL_DATA == 0)
                    dataByte &= 0xFE;
                else
                    dataByte |= 0x01;

                while (SERIAL_CLK);
            }
            rxBuffer[bufferIndex++] = dataByte;
        }
        rxBuffer[0] = lengthByte;
    }
    
    // Exit RX mode
    halSpiStrobe(CCxxx0_SIDLE);

    if (crcEnable) {
    
        //Calculate CRC
        checksum = CRC_INIT;

        for(i = 0; i < bufferIndex; i++)
            checksum = culCalcCRC(rxBuffer[i], checksum);
    
        if(checksum == 0)
            return(TRUE);
        else
            return(FALSE);
    } else
        return TRUE;

}// halRfReceivePacketSerial




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: RfReceivePacketSerial.c,v $
 * Revision 1.1  2005/11/09 09:40:17  sna
 * Initial version in CVS.
 *
 *
 *
 ******************************************************************************************************/