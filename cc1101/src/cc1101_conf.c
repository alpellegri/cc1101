#include "common.h"
#include "halRf.h"
#include "halSpi.h"

typedef struct {
  WORD addr;
  BYTE data;
} registerSetting_t;

// Address Config = No address check
// Base Frequency = 433.919830
// CRC Autoflush = false
// CRC Enable = true
// Carrier Frequency = 433.919830
// Channel Number = 0
// Channel Spacing = 199.951172
// Data Format = Normal mode
// Data Rate = 4.79794
// Deviation = 5.157471
// Device Address = 0
// Manchester Enable = true
// Modulation Format = ASK/OOK
// PA Ramping = false
// Packet Length = 255
// Packet Length Mode = Variable packet length mode. Packet length configured by
// the first byte after sync word Preamble Count = 4 RX Filter BW = 135.416667
// Sync Word Qualifier Mode = 30/32 sync word bits detected
// TX Power = 10
// Whitening = false
// PA table
#define PA_TABLE                                                               \
  { 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

static const registerSetting_t preferredSettings[] = {
  {CC1101_IOCFG0,      0x06},
  {CC1101_FIFOTHR,     0x47},
  {CC1101_SYNC1,       0xAA},
  {CC1101_SYNC0,       0xAA},
  {CC1101_PKTCTRL0,    0x01},
  {CC1101_FSCTRL1,     0x06},
  {CC1101_FREQ2,       0x10},
  {CC1101_FREQ1,       0xB0},
  {CC1101_FREQ0,       0x71},
  {CC1101_MDMCFG4,     0x98},
  {CC1101_MDMCFG3,     0x83},
  {CC1101_MDMCFG2,     0x31},
  {CC1101_DEVIATN,     0x15},
  {CC1101_MCSM0,       0x18},
  {CC1101_FOCCFG,      0x16},
  {CC1101_WORCTRL,     0xFB},
  {CC1101_FREND0,      0x11},
  {CC1101_FSCAL3,      0xE9},
  {CC1101_FSCAL2,      0x2A},
  {CC1101_FSCAL1,      0x00},
  {CC1101_FSCAL0,      0x1F},
  {CC1101_TEST2,       0x81},
  {CC1101_TEST1,       0x35},
  {CC1101_TEST0,       0x09},
};

void registerConfig(void) {
  BYTE writeByte;
#ifdef PA_TABLE
  BYTE paTable[] = PA_TABLE;
#endif

  // reset radio
  halSpiStrobe(CC1101_SRES);
  // write registers to radio
  for (WORD i = 0; i < (sizeof preferredSettings / sizeof(registerSetting_t));
       i++) {
    writeByte = preferredSettings[i].data;
    // halSpiWriteBurstReg(preferredSettings[i].addr, &writeByte, 1);
    halSpiWriteReg(preferredSettings[i].addr, writeByte);
  }
#ifdef PA_TABLE
  // write PA_TABLE
  halSpiWriteBurstReg(CC1101_PATABLE, paTable, sizeof(paTable));
#endif

#if 0
  // halSpiWriteReg(CC1101_SYNC1, 0x55);
  // halSpiWriteReg(CC1101_SYNC0, 0x55);
  halSpiWriteReg(CC1101_PKTLEN, 0xFF);
  halSpiWriteReg(CC1101_IOCFG2, 0x0B);
  halSpiWriteReg(CC1101_IOCFG0, 0x06);
  halSpiWriteReg(CC1101_PKTCTRL0, 0x02);
  halSpiWriteReg(CC1101_PKTCTRL1, 0x00);
  halSpiWriteReg(CC1101_MDMCFG2, 0x08);
  halSpiWriteReg(CC1101_MCSM1, 0x00);
#endif
}
