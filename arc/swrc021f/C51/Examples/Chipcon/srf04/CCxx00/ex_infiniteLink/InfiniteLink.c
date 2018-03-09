/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This program lets the user test a simple rf link with packets longer than  *
 *      ***   +             256 bytes.                                                                 *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     InfiniteLink.c                                                             *
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
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>
#include <InfiniteLink.h>
#include <stdlib.h>
#include <intrins.h>




//-------------------------------------------------------------------------------------------------------
// DESCRIPTION:
//      This program demonstrates how it is possible to transmit and receive packets that are longer 
//      than 256 bytes. The radios use a combination of infinite and fixed packet length mode,
//      CRC enabled and append status. No form of filtering should be used as one should not read any
//      status registers (See the CC1100 or 2500 Errata Note) to determine if the FIFO 
//      has been flushed or not. 
//      If other data rates than 76.8 kbps should be used, the halWait(320) in the EXTERNAL0_ISR(void)
//      needs to be changed to:
//      halWait(n);, where n = 3*8*(1/data rate) + a couple of us margin
//      For better understanding on how this program works, please see the 
//      CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual   
//
//      Parameters          | Values 
//      ------------------------------------------------------
//      Packet Length       | 270, 290,....., 430, 450
//      Number of Packets   | 100, 200, 300, ......, 900, 1000
//      Radio Mode          | Rx, Tx
//
//      RX UNIT: 
//                  * Set radio mode to RX. 
//                  * Move joystick down until the message 
//                    "Press S1 to start" is showed on the LCD display  
//                  * Press S1
//                  
//                  The LCD display will show number of packets received with CRC OK
//
//      TX UNIT:
//                  * Set packet length and number of packets to transmit
//                  * Set radio mode to TX
//                  * Press S1 to Start
//
//                  The LCD will show number of packets transmitted
//
//                  After all the packets have been transmitted, S1 can be pressed again to transmit 
//                  or the joystick can be used to change packet length and number of packets 
//                  to transmit 
//
//  
//      IMPORTANT NOTICE!
//      When compiling this example you will get the following warnings:
//
//      *** WARNING L15: MULTIPLE CALL TO FUNCTION
//          NAME:    _HALSPIREADBURSTREG/SPIREADBURSTREG
//          CALLER1: EXTERNAL1_ISR/INFINITELINK
//          CALLER2: EXTERNAL0_ISR/INFINITELINK
//      *** WARNING L15: MULTIPLE CALL TO FUNCTION
//          NAME:    _HALSPIWRITEBURSTREG/SPIWRITEBURSTREG
//          CALLER1: EXTERNAL1_ISR/INFINITELINK
//          CALLER2: ?C_C51STARTUP
//
//      These warnings are generated because the halSpiReadBurstReg and halSpiWriteBurstReg functions 
//      are not reentrant. The code is, however, written so that these interrupts will never occur while 
//      in the process of accessing the SPI interface, so these warnings can be ignored.   
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Global Variables

BYTE xdata txBuffer[454];   // 2 length bytes + 450 bytes payload + 2 status bytes
BYTE xdata rxBuffer[454];   // 2 length bytes + 450 bytes payload + 2 status bytes

UINT8 xdata joystickPosition = JOYSTICK_CENTER;
UINT8 xdata prevJoystickPosition = JOYSTICK_CENTER;

UINT8 state = SETUP;    // Variable used to hold the next state in the state machine in the main loop

TX_DATA xdata txData = {
    0,          // bytesLeft
    0,          // iterations 
    FALSE,      // writeRemainingDataFlag
    FALSE,      // packetSentFlag
    txBuffer,   // *pBufferIndex 
    0,          // packetsSent
    INFINITE    // pktFormat
};  

RX_DATA xdata rxData = {
    0,          // bytesLeft
    FALSE,      // packetReceivedFlag
    SYNC,       // syncOrEndOfPacket
    rxBuffer,   // *pBufferIndex
    0,          // lengthByte
    FALSE,      // crcOK
    0,          // packetsReceived
    INFINITE    // pktFormat
};

UINT8 fixedPacketLength;
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void main(void)
//
//  DESCRIPTION:
//      This function takes care of all the MCU initialization and radio configuration. The main loop
//      is implemented as a state machine and a state diagram can be found in the
//      CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual     
//----------------------------------------------------------------------------------------------------
void main (void) {

    #ifdef STAND_ALONE
    
		// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
    	// SYSCLK = 4X Clock Multiplier / 2     
     	CLOCK_INIT();
    #endif

	// Set up the crossbar and I/O ports to communicate with the SmartRF04EB peripherals
    IO_PORT_INIT();

    // Initialize the LCD display. The SMBus uses timer 0 to generate SCL
    ebLcdInit();

    // Initialize the ADC converter
    ebAdcInit(ADC_JOY);

    SPI_INIT(SCLK_6_MHZ);
   
    // Reset CC2500 and write rf setting to config registers
    POWER_UP_RESET_CCxxx0();

    halRfWriteRfSettings(&rfSettings);

    halSpiWriteReg(CCxxx0_PATABLE, paTable);
	
    handleAndDisplayMenu();

    INT_PRIORITY(INUM_EXTERNAL1, INT_HIGH); 
    
    // Infinite loop
    while (TRUE) {
        switch (state) {
            
            //----------------------------------------------------------------------------------------
            case SETUP:
            //----------------------------------------------------------------------------------------
            
                // This wait is implemented to give the LCD display enought time to be update
                // between each iteration
                halWait(250);

                joystickPosition = ebGetJoystickPosition();

                if (prevJoystickPosition != joystickPosition)
                    parseMenu(joystickPosition);

                prevJoystickPosition = joystickPosition;

                if ((index == START) && (ebButtonPushed())) {
                    
                    ENABLE_GLOBAL_INT(INT_OFF);

                    if (menuData.radioMode == RX) {
                        state = RX_START;
                        ebLcdUpdate("Waiting for", "data");

                        // Associated to the RX FIFO: Asserts when RX FIFO is filled above RXFIFO_THR.
                        // De-asserts when RX FIFO is drained below RXFIFO_THR.
                        halSpiWriteReg(CCxxx0_IOCFG2, 0x00);

                        SETUP_GDO2_INT(EDGE, HIGH);     // Enables external interrupt on rising edge
                        SETUP_GDO0_INT(EDGE, HIGH);
                    } else {
                        state = TX_START;
                        
                        // Associated to the TX FIFO: Asserts when the TX FIFO is filled above TXFIFO_THR.
                        // De-asserts when the TX FIFO is below TXFIFO_THR.
                        halSpiWriteReg(CCxxx0_IOCFG2, 0x02);
          
                        SETUP_GDO2_INT(EDGE, LOW);      // Enables external interrupt on falling edge
                        SETUP_GDO0_INT(EDGE, LOW);
                    }
                    _nop_(); // If this instruction is removed, IE0 might not be cleared
                    INT_SETFLAG(INUM_EXTERNAL0, INT_CLR);   // Clears the interrupt flag
                    INT_SETFLAG(INUM_EXTERNAL1, INT_CLR);   // Clears the interrupt flag
                    ENABLE_GLOBAL_INT(INT_ON);
                }
                break;            
                
            //----------------------------------------------------------------------------------------
            case TX_START:
            //----------------------------------------------------------------------------------------

                // Create data packet
                createDataPacket();

                txData.pktFormat = INFINITE;

                halWait(25000); // Delay to allow for the LCD display to be updated for every packet 
				
                txData.bytesLeft = menuData.packetLength + 2;
                fixedPacketLength = txData.bytesLeft % (MAX_VARIABLE_LENGTH + 1); 
                halSpiWriteBurstReg(CCxxx0_TXFIFO, txBuffer, FIFO_SIZE); // Fill up the TX FIFO
                halSpiStrobe(CCxxx0_STX);
                txData.bytesLeft -= FIFO_SIZE;
                txData.pBufferIndex = txBuffer + FIFO_SIZE;
                txData.iterations = (txData.bytesLeft / AVAILABLE_BYTES_IN_TX_FIFO);
                halSpiWriteReg(CCxxx0_PKTLEN, fixedPacketLength);
                       
                // Enable external interrupt 1 (the TX FIFO will need to be re-filled in this ISR)              
                ENABLE_GLOBAL_INT(INT_OFF);
                INT_ENABLE(INUM_EXTERNAL1, INT_ON);
                INT_SETFLAG(INUM_EXTERNAL1, INT_CLR);
                ENABLE_GLOBAL_INT(INT_ON);
               
                state = TX_WAIT;

            //----------------------------------------------------------------------------------------
            case TX_WAIT:
            //----------------------------------------------------------------------------------------

                if (txData.packetSentFlag) {
                    txData.packetSentFlag = FALSE;
                    intToAscii(++txData.packetsSent);
                    ebLcdUpdate("Sent:", asciiString);
                    if (txData.packetsSent < menuData.numberOfPackets)
                        state = TX_START;
                    else {
                        txData.packetsSent = 0;
                        state = SETUP;
                    }
                }
                break;
            
            //----------------------------------------------------------------------------------------
            case RX_START:
            //----------------------------------------------------------------------------------------
                halSpiStrobe(CCxxx0_SRX);
				state = RX_WAIT;

            //----------------------------------------------------------------------------------------
            case RX_WAIT:
            //----------------------------------------------------------------------------------------

                if (rxData.packetReceivedFlag) {
                    rxData.packetReceivedFlag = FALSE;
                    rxData.pBufferIndex = rxBuffer; 
                    if (rxData.crcOK) {
                        intToAscii(++rxData.packetsReceived);
                        ebLcdUpdate("Received:", asciiString);
                    }
     
                    state = RX_START;
                    
                    // Configure external interrupt 0 to give an interrupt on rising edge (sync received)
                    ENABLE_GLOBAL_INT(INT_OFF);   
                    SETUP_GDO0_INT(EDGE, HIGH);
                    _nop_(); // If this instruction is removed, IE0 might not be cleared
                    INT_SETFLAG(INUM_EXTERNAL0, INT_CLR);
                    ENABLE_GLOBAL_INT(INT_ON);
                }
                break;

            //----------------------------------------------------------------------------------------
            default:
            //----------------------------------------------------------------------------------------
                
                break;
        }
    }
}




//-------------------------------------------------------------------------------------------------------
//  void EXTERNAL1_ISR(void)
//
//  DESCRIPTION:
//      In TX mode there will be an interrupt when the number of bytes in the TX FIFO goes below the
//      threshold (less than 5 bytes left in the TX FIFO) and in RX mode there will be an interrupt when 
//      there are 60 bytes in the RX FIFO.
//      For better understanding on how this interrupt work, please see the 
//      CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual     
//-------------------------------------------------------------------------------------------------------
void EXTERNAL1_ISR(void) interrupt INUM_EXTERNAL1 {

    //----------------------- TX MODE -----------------------
    if (menuData.radioMode == TX) {
        if (txData.writeRemainingDataFlag) { // Less than 60 bytes to write to the TX FIFO

            halSpiWriteBurstReg(CCxxx0_TXFIFO, txData.pBufferIndex, txData.bytesLeft);
                
            INT_ENABLE(INUM_EXTERNAL1, INT_OFF);
                
        } else {
            halSpiWriteBurstReg(CCxxx0_TXFIFO, txData.pBufferIndex, AVAILABLE_BYTES_IN_TX_FIFO);

            // Change to fixed packet length mode when there is less than 256 bytes left to transmit
            if ((txData.bytesLeft < (MAX_VARIABLE_LENGTH + 1 - BYTES_IN_TX_FIFO)) && (txData.pktFormat == INFINITE)) {
                halSpiWriteReg(CCxxx0_PKTCTRL0, FIXED_PACKET_LENGTH);
                txData.pktFormat = FIXED;
            }

            txData.pBufferIndex += AVAILABLE_BYTES_IN_TX_FIFO;
            txData.bytesLeft -= AVAILABLE_BYTES_IN_TX_FIFO;
            if (!(--txData.iterations))
                txData.writeRemainingDataFlag = TRUE;
        }
    //-------------------------------------------------------

    //----------------------- RX MODE -----------------------
    } else {

        // Change to fixed packet length mode when there is less than 256 bytes left to receive
        if (((rxData.bytesLeft - BYTES_IN_RX_FIFO) < (MAX_VARIABLE_LENGTH + 1)) && (rxData.pktFormat == INFINITE)) {
            halSpiWriteReg(CCxxx0_PKTCTRL0, FIXED_PACKET_LENGTH);
            rxData.pktFormat = FIXED;
        }

        // Do not empty the FIFO (See the CC1100 or 2500 Errata Note)
        halSpiReadBurstReg(CCxxx0_RXFIFO, rxData.pBufferIndex, (BYTES_IN_RX_FIFO - 1));
        
        rxData.bytesLeft -= (BYTES_IN_RX_FIFO - 1);    
        rxData.pBufferIndex += (BYTES_IN_RX_FIFO - 1);
    }
    //-------------------------------------------------------

    IE1 = 0; // Clear the interrupt flag in case the interrupt flag were set while in the ISR

}// EXTERNAL_ISR




//-------------------------------------------------------------------------------------------------------
//  void EXTERNAL0_ISR(void)
//
//  DESCRIPTION:
//      In TX mode, this ISR will run when a packet has been sent. In RX mode it will run both when sync
//      is received (rising edge) and when the whole packet is received (falling edge).
//      For better understanding on how this interrupt work, please see the 
//      CC1100/CC1150DK & CC2500/CC2550DK Development Kit Examples and Libraries User Manual 
//-------------------------------------------------------------------------------------------------------
void EXTERNAL0_ISR(void) interrupt INUM_EXTERNAL0 {

    //----------------------- TX MODE -----------------------
    if (menuData.radioMode == TX) {    
        txData.writeRemainingDataFlag = FALSE;
        txData.packetSentFlag = TRUE;
        halSpiWriteReg(CCxxx0_PKTCTRL0, INFINITE_PACKET_LENGTH);
    //-------------------------------------------------------

    //----------------------- RX MODE -----------------------
    } else {
        if (rxData.syncOrEndOfPacket == SYNC) {

            // After the sync word is received one needs to wait some time before there will be any data 
            // in the FIFO.In addition, the FIFO should not be emptied 
            // (See the CC1100 or 2500 Errata Note) before the whole packet has been received.
            halWait(320); // Allow for 3 bytes to be put in the FIFO (3*8*(1/76800)) = 312.5 us
                
            rxData.lengthByte = halSpiReadReg(CCxxx0_RXFIFO) << 8;
            rxData.lengthByte |= halSpiReadReg(CCxxx0_RXFIFO);

            if ((rxData.lengthByte > MAX_VARIABLE_LENGTH) && (rxData.lengthByte <= menuTable[PACKET_LENGTH].max)) {
                rxData.bytesLeft = rxData.lengthByte + 2;
                fixedPacketLength = rxData.bytesLeft  % (MAX_VARIABLE_LENGTH + 1);
                halSpiWriteReg(CCxxx0_PKTLEN, fixedPacketLength);
                rxData.pBufferIndex += 2;
                rxData.syncOrEndOfPacket = END_OF_PACKET;
                            
                SETUP_GDO0_INT(EDGE, LOW);  // Enables external interrupt on falling edge (packet received)
                _nop_();                    // If this instruction is removed, IE0 might not be cleared
                INT_SETFLAG(INUM_EXTERNAL0, INT_CLR);
            } else {
                halSpiStrobe(CCxxx0_SIDLE);
                halSpiStrobe(CCxxx0_SFRX);
                rxData.syncOrEndOfPacket = SYNC;
                rxData.pktFormat = INFINITE;
                halSpiWriteReg(CCxxx0_PKTCTRL0, INFINITE_PACKET_LENGTH);

                // Configure external interrupt 0 to give an interrupt on rising edge (sync received)
                ENABLE_GLOBAL_INT(INT_OFF);   
                SETUP_GDO0_INT(EDGE, HIGH);
                _nop_(); // If this instruction is removed, IE0 might not be cleared
                INT_SETFLAG(INUM_EXTERNAL0, INT_CLR);
                ENABLE_GLOBAL_INT(INT_ON);
                state = RX_START;
            }            
        } else { // End of Packet
            halSpiReadBurstReg(CCxxx0_RXFIFO, rxData.pBufferIndex, rxData.bytesLeft); 
            rxBuffer[0] = rxData.lengthByte >> 8;
            rxBuffer[1] = rxData.lengthByte;
            rxData.syncOrEndOfPacket = SYNC;
            rxData.packetReceivedFlag = TRUE;
            rxData.crcOK = ((rxBuffer[rxData.lengthByte + 3]) & CRC_OK);
            rxData.pktFormat = INFINITE;
            halSpiWriteReg(CCxxx0_PKTCTRL0, INFINITE_PACKET_LENGTH);
        }
    //-------------------------------------------------------  
    }
}// EXTERNAL_ISR




//-------------------------------------------------------------------------------------------------------
//  void intToAscii(UINT16 value)
//
//  DESCRIPTION:
//		Takes a 16 bits integer as input and converts it to ascii. The result is put in a global
//      variable (UINT8 xdata asciiString[6])
//
//	ARGUMENTS:
//		UINT16 value
//			The value to be converted
//-------------------------------------------------------------------------------------------------------
void intToAscii(UINT16 value) {
    UINT8 i;
    UINT8 j = 0;
    UINT8 digit_start = 0;
    UINT16 digit = 0;
    UINT16 denom = 10000;

    if (value == 0) {
        asciiString[0] = '0';
        asciiString[1] = NULL;
    } else {
        for(i = 5; i > 0; i--) {
            digit = value / denom;
            if((digit_start == 1) || (digit != 0)) {
                digit_start = 1;
                value %= denom;
                asciiString[j++] = (digit + '0');
            }
            denom /= 10;
        }
        asciiString[j++] = NULL;
    }
}// intToAscii



//-------------------------------------------------------------------------------------------------------
//  void createDataPacket(void) 
//
//  DESCRIPTION: 
//      This function is called before a packet is going to be transmitted.
//      Packet format (length byte >= 255):
//
//  |----------------------------------------------------------------------------------------------------
//  |              |              |             |             |            |             |             |
//  | Length field | Length field | Random Data | Random Data |............| Random Data | Random Data |
//  |     MSB      |     LSB      |             |             |            |             |             |
//  |----------------------------------------------------------------------------------------------------
//          ^             ^              ^                                                      ^
//          |             |              |                                                      |
//     txBuffer[0]    txBuffer[1]   txBuffer[2]                          txBuffer[menuData.packetLength]
//-------------------------------------------------------------------------------------------------------
void createDataPacket(void) {
    UINT16 xdata i;

    txBuffer[0] = menuData.packetLength >> 8;
    txBuffer[1] = menuData.packetLength;;
       
    for (i = 2; i <= (menuData.packetLength + 1); i++)
        txBuffer[i] = rand();
} //createDataPacket




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: InfiniteLink.c,v $
 * Revision 1.3  2007/11/23 09:24:03  a0190596
 * Removed halSpiWriteReg(CCxxx0_FIFOTHR, 0x0E);
 * since this register is included in the halRfWriteRfSettings(&rfSettings) function.
 *
 * Revision 1.2  2006/05/03 12:03:25  a0190596
 * no message
 *
 * Revision 1.1  2006/04/25 15:00:38  a0190596
 * Initial version in CVS.
 *
 *
 *
 *
 ******************************************************************************************************/

