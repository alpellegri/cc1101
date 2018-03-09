#include "cc1101_conf.h"
#include "cc1101_spi.h"

typedef struct {
  uint16_t addr;
  uint8_t data;
} registerSetting_t;

#define PA_TABLE                                                               \
  { 0x50, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

static const registerSetting_t preferredSettings[] = {
    {CC1101_IOCFG0, 0x06},  {CC1101_FIFOTHR, 0x47}, {CC1101_PKTCTRL0, 0x05},
    {CC1101_FSCTRL1, 0x06}, {CC1101_FREQ2, 0x10},   {CC1101_FREQ1, 0xB0},
    {CC1101_FREQ0, 0x71},   {CC1101_MDMCFG4, 0xA7}, {CC1101_MDMCFG3, 0x83},
    {CC1101_MDMCFG2, 0x3B}, {CC1101_DEVIATN, 0x15}, {CC1101_MCSM0, 0x18},
    {CC1101_FOCCFG, 0x16},  {CC1101_WORCTRL, 0xFB}, {CC1101_FREND0, 0x11},
    {CC1101_FSCAL3, 0xE9},  {CC1101_FSCAL2, 0x2A},  {CC1101_FSCAL1, 0x00},
    {CC1101_FSCAL0, 0x1F},  {CC1101_TEST2, 0x81},   {CC1101_TEST1, 0x35},
    {CC1101_TEST0, 0x09},
};

void registerConfig(void) {
  uint8_t writeByte;
#ifdef PA_TABLE
  uint8_t paTable[] = PA_TABLE;
#endif

  // reset radio
  trxSpiCmdStrobe(CC1101_SRES);
  // write registers to radio
  for (uint16_t i = 0;
       i < (sizeof preferredSettings / sizeof(registerSetting_t)); i++) {
    writeByte = preferredSettings[i].data;
    cc1101SpiWriteReg(preferredSettings[i].addr, &writeByte, 1);
  }
#ifdef PA_TABLE
  // write PA_TABLE
  cc1101SpiWriteReg(CC1101_PA_TABLE0, paTable, sizeof(paTable));
#endif
}
