/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file register settings for the 5 different frequency band             *
 *      ***   +             supported by CCxx00.                                                       *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     RegSettings_SerialLink.c                                                   *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:                Keil C51 V7.50                                                             *
 * Target platform:         Chipcon CCxxx0 (Silabs F320)                                               *
 * Author:                  SNA                                                                        *
 *******************************************************************************************************
 * Revision history:     See end of file                                                               *
 *******************************************************************************************************/
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\halsrf04.h>
#include <Chipcon\srf04\ebsrf04.h>




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_315_CC1100
// Chipcon
// Product = CC1100
// Chip version = F   (VERSION = 0x03)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (0) 2-FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 314.999664 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x0C,   // FREQ2     Frequency control word, high byte.
    0x1D,   // FREQ1     Frequency control word, middle byte.
    0x89,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x00,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x0B,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration. 
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x60;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_315_CC1101
// Chipcon
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 314.999664 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x0C,   // FREQ2     Frequency control word, high byte.
    0x1D,   // FREQ1     Frequency control word, middle byte.
    0x89,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x10,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x0B,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x51;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_433_CC1100
// Chipcon
// Product = CC1100
// Chip version = F   (VERSION = 0x03)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (0) 2-FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 432.999817 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x10,   // FREQ2     Frequency control word, high byte.
    0xA7,   // FREQ1     Frequency control word, middle byte.
    0x62,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x00,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration. 
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};


// PATABLE (0 dBm output power)
BYTE code paTable = 0x60;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_433_CC1101
// Chipcon
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 432.999817 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x10,   // FREQ2     Frequency control word, high byte.
    0xA7,   // FREQ1     Frequency control word, middle byte.
    0x62,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x10,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x60;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_480_CC1100E
// Chipcon
// Product = CC1100E
// Chip version = A   (VERSION = 0x05)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 10 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 479.999847 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x12,   // FREQ2     Frequency control word, high byte.
    0x76,   // FREQ1     Frequency control word, middle byte.
    0x27,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x10,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x0A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (+10 dBm output power)
BYTE code paTable = 0xC2;

#endif
//-------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_868_CC1100
// Chipcon
// Product = CC1100
// Chip version = F   (VERSION = 0x03)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (0) 2-FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 867.999939 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x21,   // FREQ2     Frequency control word, high byte.
    0x62,   // FREQ1     Frequency control word, middle byte.
    0x76,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x00,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration. 
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x8E;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_868_CC1101
// Chipcon
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 867.999939 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x21,   // FREQ2     Frequency control word, high byte.
    0x62,   // FREQ1     Frequency control word, middle byte.
    0x76,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x10,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x50;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_915_CC1100
// Chipcon
// Product = CC1100
// Chip version = F   (VERSION = 0x03)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (0) 2-FSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 914.999969 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x23,   // FREQ2     Frequency control word, high byte.
    0x31,   // FREQ1     Frequency control word, middle byte.
    0x3B,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x00,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x8E;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_915_CC1101
// Chipcon
// Product = CC1101
// Chip version = A   (VERSION = 0x04)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 325.000000 kHz
// Deviation = 48 kHz
// Datarate = 99.975586 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 914.999969 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x23,   // FREQ2     Frequency control word, high byte.
    0x31,   // FREQ1     Frequency control word, middle byte.
    0x3B,   // FREQ0     Frequency control word, low byte.
    0x5B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x10,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x47,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x2A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x1F,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x8E;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_955_CC1100E
// Chipcon
// Product = CC1100E
// Chip version = A   (VERSION = 0x05)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 232.142857 kHz
// Deviation = 32 kHz
// Datarate = 76.766968 kBaud
// Modulation = (1) GFSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 954.999725 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = -
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x08,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x24,   // FREQ2     Frequency control word, high byte.
    0xBB,   // FREQ1     Frequency control word, middle byte.
    0x13,   // FREQ0     Frequency control word, low byte.
    0x7B,   // MDMCFG4   Modem configuration.
    0x83,   // MDMCFG3   Modem configuration.
    0x10,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x42,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB2,   // AGCCTRL0  AGC control.
    0xAA,   // FSCAL3    Frequency synthesizer calibration.
    0x0A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x07,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x81,   // TEST2     Various test settings.
    0x35,   // TEST1     Various test settings.
    0x09,   // TEST0     Various test settings.
    0x47,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration. Refer to SmartRF® Studio User Manual for detailed pseudo register explanation.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0x60;

#endif
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#ifdef FREQ_2_4
// Chipcon
// Product = CC2500
// Chip version = E   (VERSION = 0x03)
// Crystal accuracy = 10 ppm
// X-tal frequency = 26 MHz
// RF output power = 0 dBm
// RX filterbandwidth = 541.666667 kHz
// Phase = 1
// Datarate = 99.975586 kBaud
// Modulation = (7) MSK
// Manchester enable = (0) Manchester disabled
// RF Frequency = 2432.999908 MHz
// Channel spacing = 199.951172 kHz
// Channel number = 0
// Optimization = Sensitivity
// Sync mode = (0) No preamble/sync
// Format of RX/TX data = (1) Serial Synchronous mode, used for backwards compatibility
// CRC operation = (0) CRC disabled for TX and RX
// Forward Error Correction = (0) FEC disabled
// Length configuration = (2) Enable infinite length packets.
// Packetlength = 255
// Preamble count = (2)  4 bytes
// Append status = 0
// Address check = (0) No address check
// FIFO autoflush = 0
// Device address = 0
// GDO0 signal selection = (12) Serial Synchronous Data Output
// GDO2 signal selection = (11) Serial Clock
RF_SETTINGS code rfSettings = {
    0x0A,   // FSCTRL1   Frequency synthesizer control.
    0x00,   // FSCTRL0   Frequency synthesizer control.
    0x5D,   // FREQ2     Frequency control word, high byte.
    0x93,   // FREQ1     Frequency control word, middle byte.
    0xB1,   // FREQ0     Frequency control word, low byte.
    0x2B,   // MDMCFG4   Modem configuration.
    0xF8,   // MDMCFG3   Modem configuration.
    0x70,   // MDMCFG2   Modem configuration.
    0x22,   // MDMCFG1   Modem configuration.
    0xF8,   // MDMCFG0   Modem configuration.
    0x00,   // CHANNR    Channel number.
    0x01,   // DEVIATN   Modem deviation setting (when FSK modulation is enabled).
    0xB6,   // FREND1    Front end RX configuration.
    0x10,   // FREND0    Front end TX configuration.
    0x18,   // MCSM0     Main Radio Control State Machine configuration.
    0x1D,   // FOCCFG    Frequency Offset Compensation Configuration.
    0x1C,   // BSCFG     Bit synchronization Configuration.
    0xC7,   // AGCCTRL2  AGC control.
    0x00,   // AGCCTRL1  AGC control.
    0xB0,   // AGCCTRL0  AGC control.
    0xEA,   // FSCAL3    Frequency synthesizer calibration.
    0x0A,   // FSCAL2    Frequency synthesizer calibration.
    0x00,   // FSCAL1    Frequency synthesizer calibration.
    0x11,   // FSCAL0    Frequency synthesizer calibration.
    0x59,   // FSTEST    Frequency synthesizer calibration.
    0x88,   // TEST2     Various test settings.
    0x31,   // TEST1     Various test settings.
    0x0B,   // TEST0     Various test settings.
    0x07,   // FIFOTHR   RXFIFO and TXFIFO thresholds.
    0x0B,   // IOCFG2    GDO2 output pin configuration.
    0x0C,   // IOCFG0D   GDO0 output pin configuration.
    0x00,   // PKTCTRL1  Packet automation control.
    0x12,   // PKTCTRL0  Packet automation control.
    0x00,   // ADDR      Device address.
    0xFF    // PKTLEN    Packet length.
};

// PATABLE (0 dBm output power)
BYTE code paTable = 0xFE;

#endif
//------------------------------------------------------------------------------------------------------



/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: RegSettings_SerialLink.c,v $
 * Revision 1.6  2007/11/23 09:56:17  a0190596
 * Updated in accordance with SmartRF Studio settings
 *
 * Revision 1.5  2007/06/22 14:23:04  a0190596
 * Changed register settings according to new characterization results
 *
 * Revision 1.4  2007/01/23 07:38:54  a0190596
 * Changed register settings according to new characterization results
 *
 * Revision 1.3  2006/11/17 13:13:54  a0190596
 * no message
 *
 * Revision 1.2  2006/11/14 13:41:42  a0190596
 * Changed PATABLE settings for 0 dBm
 *
 * Revision 1.1  2006/03/17 10:24:23  a0190596
 * Initial version in CVS.
 * Will replace the .h file used in previous examples
 *
 *
 *
 *
 ******************************************************************************************************/

