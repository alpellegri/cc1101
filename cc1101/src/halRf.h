#ifndef HALRF_H
#define HALRF_H

#include "common.h"

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------------------------------------------------------------------------------------
// Defines
#define CRC_OK 0x80
#define GDO0_PIN P0_6
#define RSSI 0
#define LQI 1
#define BYTES_IN_RXFIFO 0x7F
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Definitions to support burst/single access:
#define WRITE_BURST 0x40
#define READ_SINGLE 0x80
#define READ_BURST 0xC0
//-------------------------------------------------------------------------------------------------------

//------------------------------------------------------------------------------------------------------
// CC2500/CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CC1101_IOCFG2 0x00   // GDO2 output pin configuration
#define CC1101_IOCFG1 0x01   // GDO1 output pin configuration
#define CC1101_IOCFG0 0x02   // GDO0 output pin configuration
#define CC1101_FIFOTHR 0x03  // RX FIFO and TX FIFO thresholds
#define CC1101_SYNC1 0x04    // Sync word, high byte
#define CC1101_SYNC0 0x05    // Sync word, low byte
#define CC1101_PKTLEN 0x06   // Packet length
#define CC1101_PKTCTRL1 0x07 // Packet automation control
#define CC1101_PKTCTRL0 0x08 // Packet automation control
#define CC1101_ADDR 0x09     // Device address
#define CC1101_CHANNR 0x0A   // Channel number
#define CC1101_FSCTRL1 0x0B  // Frequency synthesizer control
#define CC1101_FSCTRL0 0x0C  // Frequency synthesizer control
#define CC1101_FREQ2 0x0D    // Frequency control word, high byte
#define CC1101_FREQ1 0x0E    // Frequency control word, middle byte
#define CC1101_FREQ0 0x0F    // Frequency control word, low byte
#define CC1101_MDMCFG4 0x10  // Modem configuration
#define CC1101_MDMCFG3 0x11  // Modem configuration
#define CC1101_MDMCFG2 0x12  // Modem configuration
#define CC1101_MDMCFG1 0x13  // Modem configuration
#define CC1101_MDMCFG0 0x14  // Modem configuration
#define CC1101_DEVIATN 0x15  // Modem deviation setting
#define CC1101_MCSM2 0x16    // Main Radio Control State Machine configuration
#define CC1101_MCSM1 0x17    // Main Radio Control State Machine configuration
#define CC1101_MCSM0 0x18    // Main Radio Control State Machine configuration
#define CC1101_FOCCFG 0x19   // Frequency Offset Compensation configuration
#define CC1101_BSCFG 0x1A    // Bit Synchronization configuration
#define CC1101_AGCCTRL2 0x1B // AGC control
#define CC1101_AGCCTRL1 0x1C // AGC control
#define CC1101_AGCCTRL0 0x1D // AGC control
#define CC1101_WOREVT1 0x1E  // High byte Event 0 timeout
#define CC1101_WOREVT0 0x1F  // Low byte Event 0 timeout
#define CC1101_WORCTRL 0x20  // Wake On Radio control
#define CC1101_FREND1 0x21   // Front end RX configuration
#define CC1101_FREND0 0x22   // Front end TX configuration
#define CC1101_FSCAL3 0x23   // Frequency synthesizer calibration
#define CC1101_FSCAL2 0x24   // Frequency synthesizer calibration
#define CC1101_FSCAL1 0x25   // Frequency synthesizer calibration
#define CC1101_FSCAL0 0x26   // Frequency synthesizer calibration
#define CC1101_RCCTRL1 0x27  // RC oscillator configuration
#define CC1101_RCCTRL0 0x28  // RC oscillator configuration
#define CC1101_FSTEST 0x29   // Frequency synthesizer calibration control
#define CC1101_PTEST 0x2A    // Production test
#define CC1101_AGCTEST 0x2B  // AGC test
#define CC1101_TEST2 0x2C    // Various test settings
#define CC1101_TEST1 0x2D    // Various test settings
#define CC1101_TEST0 0x2E    // Various test settings

// Strobe commands
#define CC1101_SRES 0x30 // Reset chip.
#define CC1101_SFSTXON                                                         \
  0x31 // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
       // If in RX/TX: Go to a wait state where only the synthesizer is
       // running (for quick RX / TX turnaround).
#define CC1101_SXOFF 0x32 // Turn off crystal oscillator.
#define CC1101_SCAL                                                            \
  0x33 // Calibrate frequency synthesizer and turn it off
       // (enables quick start).
#define CC1101_SRX                                                             \
  0x34 // Enable RX. Perform calibration first if coming from IDLE and
       // MCSM0.FS_AUTOCAL=1.
#define CC1101_STX                                                             \
  0x35 // In IDLE state: Enable TX. Perform calibration first if
       // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
       // Only go to TX if channel is clear.
#define CC1101_SIDLE                                                           \
  0x36 // Exit RX / TX, turn off frequency synthesizer and exit
       // Wake-On-Radio mode if applicable.
#define CC1101_SAFC 0x37 // Perform AFC adjustment of the frequency synthesizer
#define CC1101_SWOR 0x38 // Start automatic RX polling sequence (Wake-on-Radio)
#define CC1101_SPWD 0x39 // Enter power down mode when CSn goes high.
#define CC1101_SFRX 0x3A // Flush the RX FIFO buffer.
#define CC1101_SFTX 0x3B // Flush the TX FIFO buffer.
#define CC1101_SWORRST 0x3C // Reset real time clock.
#define CC1101_SNOP                                                            \
  0x3D // No operation. May be used to pad strobe commands to two
       // bytes for simpler software.

#define CC1101_PARTNUM 0x30
#define CC1101_VERSION 0x31
#define CC1101_FREQEST 0x32
#define CC1101_LQI 0x33
#define CC1101_RSSI 0x34
#define CC1101_MARCSTATE 0x35
#define CC1101_WORTIME1 0x36
#define CC1101_WORTIME0 0x37
#define CC1101_PKTSTATUS 0x38
#define CC1101_VCO_VC_DAC 0x39
#define CC1101_TXBYTES 0x3A
#define CC1101_RXBYTES 0x3B
#define CC1101_RCCTRL1_STATUS 0x3C
#define CC1101_RCCTRL0_STATUS 0x3D

#define CC1101_PATABLE 0x3E
#define CC1101_TXFIFO 0x3F
#define CC1101_RXFIFO 0x3F

//-------------------------------------------------------------------------------------------------------
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS {
  BYTE FSCTRL1;  // Frequency synthesizer control.
  BYTE FSCTRL0;  // Frequency synthesizer control.
  BYTE FREQ2;    // Frequency control word, high byte.
  BYTE FREQ1;    // Frequency control word, middle byte.
  BYTE FREQ0;    // Frequency control word, low byte.
  BYTE MDMCFG4;  // Modem configuration.
  BYTE MDMCFG3;  // Modem configuration.
  BYTE MDMCFG2;  // Modem configuration.
  BYTE MDMCFG1;  // Modem configuration.
  BYTE MDMCFG0;  // Modem configuration.
  BYTE CHANNR;   // Channel number.
  BYTE DEVIATN;  // Modem deviation setting (when FSK modulation is enabled).
  BYTE FREND1;   // Front end RX configuration.
  BYTE FREND0;   // Front end RX configuration.
  BYTE MCSM0;    // Main Radio Control State Machine configuration.
  BYTE FOCCFG;   // Frequency Offset Compensation Configuration.
  BYTE BSCFG;    // Bit synchronization Configuration.
  BYTE AGCCTRL2; // AGC control.
  BYTE AGCCTRL1; // AGC control.
  BYTE AGCCTRL0; // AGC control.
  BYTE FSCAL3;   // Frequency synthesizer calibration.
  BYTE FSCAL2;   // Frequency synthesizer calibration.
  BYTE FSCAL1;   // Frequency synthesizer calibration.
  BYTE FSCAL0;   // Frequency synthesizer calibration.
  BYTE FSTEST;   // Frequency synthesizer calibration control
  BYTE TEST2;    // Various test settings.
  BYTE TEST1;    // Various test settings.
  BYTE TEST0;    // Various test settings.
  BYTE FIFOTHR;  // RXFIFO and TXFIFO thresholds.
  BYTE IOCFG2;   // GDO2 output pin configuration
  BYTE IOCFG0;   // GDO0 output pin configuration
  BYTE PKTCTRL1; // Packet automation control.
  BYTE PKTCTRL0; // Packet automation control.
  BYTE ADDR;     // Device address.
  BYTE PKTLEN;   // Packet length.
} RF_SETTINGS;

void halRfInit(void);

//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacket(BYTE *txBuffer, UINT8 size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to
//      63 bytes. To use this function, GD00 must be configured to be asserted
//      when sync word is sent and de-asserted at the end of the packet =>
//      halSpiWriteReg(CC1101_IOCFG0, 0x06); The function implements polling of
//      GDO0. First it waits for GD00 to be set and then it waits for it to be
//      cleared.
//
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containing the data that are going to be
//          transmitted
//
//      UINT8 size
//          The size of the txBuffer
//-------------------------------------------------------------------------------------------------------
void halRfSendPacket(BYTE *txBuffer, UINT8 size);

//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION:
//      This function can be used to receive a packet of variable packet length
//      (first byte in the packet must be the length byte). The packet length
//      should not exceed the RX FIFO size. To use this function, GD00 must be
//      configured to be asserted when sync word is sent and de-asserted at the
//      end of the packet => halSpiWriteReg(CC1101_IOCFG0, 0x06); Also,
//      APPEND_STATUS in the PKTCTRL1 register must be enabled. The function
//      implements polling of GDO0. First it waits for GD00 to be set and then
//      it waits for it to be cleared. After the GDO0 pin has been de-asserted,
//      the RXBYTES register is read to make sure that there are bytes in the
//      FIFO. This is because the GDO signal will indicate sync received even if
//      the FIFO is flushed due to address filtering, CRC filtering, or packet
//      length filtering.
//
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the
//          incoming data should be stored. After this function returns, that
//          variable holds the packet length.
//
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK (or no packet was put in the RX FIFO due to
//          filtering)
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length);

#ifdef __cplusplus
}
#endif

#endif
