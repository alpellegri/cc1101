#include "cc1101.h"

#define PA_TABLE                                                               \
  { 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }

const registerSetting_t preferredSettings[] = {
    {CC1101_IOCFG2, 0x06},   {CC1101_IOCFG0, 0x01},  {CC1101_FIFOTHR, 0x47},
    {CC1101_SYNC1, 0x55},    {CC1101_SYNC0, 0x56},   {CC1101_PKTCTRL1, 0x00},
    {CC1101_PKTCTRL0, 0x02}, {CC1101_FSCTRL1, 0x06}, {CC1101_FREQ2, 0x10},
    {CC1101_FREQ1, 0xB0},    {CC1101_FREQ0, 0x71},   {CC1101_MDMCFG4, 0x98},
    {CC1101_MDMCFG3, 0x83},  {CC1101_MDMCFG2, 0x32}, {CC1101_MDMCFG1, 0x02},
    {CC1101_DEVIATN, 0x15},  {CC1101_MCSM2, 0x17},   {CC1101_MCSM0, 0x18},
    {CC1101_FOCCFG, 0x16},   {CC1101_WORCTRL, 0xFB}, {CC1101_FREND0, 0x11},
    {CC1101_FSCAL3, 0xE9},   {CC1101_FSCAL2, 0x2A},  {CC1101_FSCAL1, 0x00},
    {CC1101_FSCAL0, 0x1F},   {CC1101_TEST2, 0x81},   {CC1101_TEST1, 0x35},
    {CC1101_TEST0, 0x09},
};

const uint8_t preferredSettings_size =
    (sizeof(preferredSettings) / sizeof(registerSetting_t));
