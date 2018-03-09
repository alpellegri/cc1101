/***********************************************************************************
    Filename: hal_rf.h

    Copyright 2007 Texas Instruments, Inc.
***********************************************************************************/

#ifndef HAL_RF_H
#define HAL_RF_H

#include <hal_types.h>

//----------------------------------------------------------------------------------
// Type declarations
//----------------------------------------------------------------------------------

// The following structure can be used when configuring the RF chip. SmartRF Studio
// can be used to export a predefined instance of this structure.
typedef struct {
    uint8 fsctrl1;    // Frequency synthesizer control.
    uint8 fsctrl0;    // Frequency synthesizer control.
    uint8 freq2;      // Frequency control word, high byte.
    uint8 freq1;      // Frequency control word, middle byte.
    uint8 freq0;      // Frequency control word, low byte.
    uint8 mdmcfg4;    // Modem configuration.
    uint8 mdmcfg3;    // Modem configuration.
    uint8 mdmcfg2;    // Modem configuration.
    uint8 mdmcfg1;    // Modem configuration.
    uint8 mdmcfg0;    // Modem configuration.
    uint8 channr;     // Channel number.
    uint8 deviatn;    // Modem deviation setting (when FSK modulation is enabled).
    uint8 frend1;     // Front end RX configuration.
    uint8 frend0;     // Front end RX configuration.
    uint8 mcsm0;      // Main Radio Control State Machine configuration.
    uint8 foccfg;     // Frequency Offset Compensation Configuration.
    uint8 bscfg;      // Bit synchronization Configuration.
    uint8 agcctrl2;   // AGC control.
    uint8 agcctrl1;   // AGC control.
    uint8 agcctrl0;   // AGC control.
    uint8 fscal3;     // Frequency synthesizer calibration.
    uint8 fscal2;     // Frequency synthesizer calibration.
    uint8 fscal1;     // Frequency synthesizer calibration.
    uint8 fscal0;     // Frequency synthesizer calibration.
    uint8 fstest;     // Frequency synthesizer calibration.
    uint8 test2;      // Various test settings.
    uint8 test1;      // Various test settings.
    uint8 test0;      // Various test settings.
    uint8 iocfg2;     // GDO2 output pin configuration.
    uint8 iocfg0;     // GDO0 output pin configuration.
    uint8 pktctrl1;   // Packet automation control.
    uint8 pktctrl0;   // Packet automation control.
    uint8 addr;       // Device address.
    uint8 pktlen;     // Packet length.
} HAL_RF_CONFIG;

// For rapid chip configuration with a minimum of function overhead.
// The array has to be set up with predefined values for all registers.
typedef uint8 HAL_RF_BURST_CONFIG[47];

// The chip status byte, returned by chip for all SPI accesses
typedef uint8 HAL_RF_STATUS;



//----------------------------------------------------------------------------------
// Function declarations
//----------------------------------------------------------------------------------

void  halRfConfig(const HAL_RF_CONFIG* rfConfig, const uint8* rfPaTable, uint8 rfPaTableLen);
void  halRfBurstConfig(const HAL_RF_BURST_CONFIG rfConfig, const uint8* rfPaTable, uint8 rfPaTableLen);
void  halRfResetChip(void);
uint8 halRfGetChipId(void);
uint8 halRfGetChipVer(void);
uint8 halRfReadStatusReg(uint8 addr);
uint8 halRfReadReg(uint8 addr);

HAL_RF_STATUS halRfWriteReg(uint8 addr, uint8 data);
HAL_RF_STATUS halRfWriteFifo(const uint8* data, uint8 length);
HAL_RF_STATUS halRfReadFifo(uint8* data, uint8 length);
HAL_RF_STATUS halRfStrobe(uint8 cmd);
HAL_RF_STATUS halRfGetTxStatus(void);
HAL_RF_STATUS halRfGetRxStatus(void);


/**********************************************************************************/
#endif
