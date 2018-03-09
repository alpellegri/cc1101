/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains macros and function prototypes for the c8051f320        *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     halsrf04.h                                                                 *
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
#ifndef HALSRF04_H
#define HALSRF04_H
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\common.h>





/*******************************************************************************************************
 *******************************************************************************************************
 **************************              Interrupt functions/macros          ***************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
//  This section contain useful macros for enable/disable interrupt, set and get flag, set interrupt
//  priority.
//      ENABLE_GLOBAL_INT(on)
//      INT_ENABLE(inum, on)
//      INT_PRIORITY(inum, p)
//      INT_GETFLAG(inum)
//      INT_SETFLAG(inum, f)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// General
#define ENABLE_GLOBAL_INT(on)   do { EA = on; } while (0)

// where on is one of:
#define INT_ON      1
#define INT_OFF     0
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro used together with the INUM_* constants defined in regssrf04.h to enable or disable certain 
// interrupts. E.g:
// INT_ENABLE(INUM_EXTERNAL0, INT_ON); 
// INT_ENABLE(INUM_UART0, INT_OFF); 
#define INT_ENABLE(inum, on) \
    do { \
        if (inum == INUM_EXTERNAL0) \
            EX0 = (on); \
        else if (inum == INUM_TIMER0) \
            ET0 = (on); \
        else if (inum == INUM_EXTERNAL1) \
            EX1 = (on); \
        else if (inum == INUM_TIMER1) \
            ET1 = (on); \
        else if (inum == INUM_UART0) \
            ES0 = (on); \
        else if (inum == INUM_TIMER2) \
            ET2 = (on); \
        else if (inum == INUM_SPI0) \
            ESPI0 = (on); \
        else if (inum == INUM_SMB0) \
            (on ? (EIE1 |= BM_ESMB0) : (EIP1 &= (~BM_ESMB0))); \
        else if (inum == INUM_USB0) \
            (on ? (EIE1 |= BM_EUSB0) : (EIE1 &= (~BM_EUSB0))); \
        else if (inum == INUM_ADC0WINDOW) \
            (on ? (EIE1 |= BM_EWADC0) : (EIE1 &= (~BM_EWADC0))); \
        else if (inum == INUM_ADC0) \
            (on ? (EIE1 |= BM_EADC0) : (EIE1 &= (~BM_EADC0))); \
        else if (inum == INUM_PCA0) \
            (on ? (EIE1 |= BM_EPCA0) : (EIE1 &= (~BM_EPCA0))); \
        else if (inum == INUM_ACOMP0) \
            (on ? (EIE1 |= BM_ECP0) : (EIE1 &= (~BM_ECP0))); \
        else if (inum == INUM_ACOMP1) \
            (on ? (EIE1 |= BM_ECP1) : (EIE1 &= (~BM_ECP1))); \
        else if (inum == INUM_TIMER3) \
            (on ? (EIE1 |= BM_ET3) : (EIE1 &= (~BM_ET3))); \
        else if (inum == INUM_VBUS) \
            (on ? (EIE2 |= BM_EVBUS) : (EIE2 &= (~BM_EVBUS))); \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro used together with the INUM_* constants defined in regssrf04.h to set the priority of certain 
// interrupts. E.g:
// INT_PRIORITY(INUM_RF, INT_HIGH);
// INT_PRIORITY(INUM_UART0, INT_LOW);
#define INT_PRIORITY(inum, p) \
    do {\
        if (inum == INUM_EXTERNAL0) \
                PX0 = (p); \
        else if (inum == INUM_TIMER0) \
                PT0 = (p); \
        else if (inum == INUM_EXTERNAL1) \
                PX1 = (p); \
        else if (inum == INUM_TIMER1) \
                PT1 = (p); \
        else if (inum == INUM_UART0) \
                PS0 = (p); \
        else if (inum == INUM_TIMER2) \
                PT2 = (p); \
        else if (inum == INUM_SPI0) \
                PSPI0 = (p); \
        else if (inum == INUM_SMB0) \
            (p ? (EIP1 |= BM_PSMB0) : (EIP1 &= (~BM_PSMB0))); \
        else if (inum == INUM_USB0) \
            (p ? (EIP1 |= BM_PUSB0) : (EIP1 &= (~BM_PUSB0))); \
        else if (inum == INUM_ADC0WINDOW) \
            (p ? (EIP1 |= BM_PWADC0) : (EIP1 &= (~BM_PWADC0))); \
        else if (inum == INUM_ADC0) \
            (p ? (EIP1 |= BM_PADC0) : (EIP1 &= (~BM_PADC0))); \
        else if (inum == INUM_PCA0) \
            (p ? (EIP1 |= BM_PPCA0) : (EIP1 &= (~BM_PPCA0))); \
        else if (inum == INUM_ACOMP0) \
            (p ? (EIP1 |= BM_PCP0) : (EIP1 &= (~BM_PCP0))); \
        else if (inum == INUM_ACOMP1) \
            (p ? (EIP1 |= BM_PCP1) : (EIP1 &= (~BM_PCP1))); \
        else if (inum == INUM_TIMER3) \
            (p ? (EIP1 |= BM_PT3) : (EIP1 &= (~BM_PT3))); \
        else if (inum == INUM_VBUS) \
            (p ? (EIP2 |= BM_PVBUS) : (EIP2 &= (~BM_PVBUS))); \
    } while (0)

// where p is one of:
#define INT_HIGH    1
#define INT_LOW     0
//-------------------------------------------------------------------------------------------------------


        

//-------------------------------------------------------------------------------------------------------
// Macro used together with the INUM_* constants defined in regssrf04.h to read the interrupt flags:
// if (INT_GETFLAG(INUM_RF))
//     ...
// while (!INT_GETFLAG(INUM_UART0_TX));
#define INT_GETFLAG(inum) ( \
    (inum == INUM_EXTERNAL0)    ? \
        IE0: \
    (inum == INUM_TIMER0)       ? \
        TF0: \
    (inum == INUM_EXTERNAL1)    ? \
        IE1: \
    (inum == INUM_TIMER1)       ? \
        TF1: \
    (inum == INUM_UART0_RX)     ? \
        RI0: \
    (inum == INUM_UART0_TX)     ? \
        TI0: \
    (inum == INUM_TIMER2_H)     ? \
        TF2H: \
    (inum == INUM_TIMER2_L)     ? \
        TF2L: \
    (inum == INUM_SPI0_IF)      ? \
        SPIF: \
    (inum == INUM_SPI0_WCOL)    ? \
        WCOL: \
    (inum == INUM_SPI0_MODF)    ? \
        MODF: \
    (inum == INUM_SPI0_RXOVRN)  ? \
        RXOVRN: \
    (inum == INUM_SMB0)         ? \
        SI: \
    (inum == INUM_ADC0WINDOW)   ? \
        AD0WINT: \
    (inum == INUM_ADC0)         ? \
        AD0INT: \
    (inum == INUM_PCA0_IF)      ? \
        CF: \
    (inum == INUM_PCA0_CCF0)    ? \
        CCF0: \
    (inum == INUM_PCA0_CCF1)    ? \
        CCF1: \
    (inum == INUM_PCA0_CCF2)    ? \
        CCF2: \
    (inum == INUM_PCA0_CCF3)    ? \
        CCF3: \
    (inum == INUM_PCA0_CCF4)    ? \
        CCF4: \
    (inum == INUM_ACOMP0_F)     ? \
        (CPT0CN & BM_CP0FIF): \
    (inum == INUM_ACOMP0_R)     ? \
        (CPT0CN & BM_CP0RIF): \
    (inum == INUM_ACOMP1_F)     ? \
        (CPT1CN & BM_CP1FIF): \
    (inum == INUM_ACOMP1_R)     ? \
        (CPT1CN & BM_CP1RIF): \
    (inum == INUM_TIMER3_H)     ? \
        (TMR3CN & BM_TF3H): \
    (inum == INUM_TIMER3_L)     ? \
        (TMR3CN & BM_TF3L): \
    0 \
)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro used together with the INUM_* constants defined in regsrf04.h to set
// or clear certain interrupt flags. E.g:
// INT_SETFLAG(INUM_UART0_RX, INT_CLR);
#define INT_SETFLAG(inum, f) \
    do { \
        if (inum == INUM_EXTERNAL0) \
            IE0 = (f); \
        else if (inum == INUM_TIMER0) \
            TF0 = (f); \
        else if (inum == INUM_EXTERNAL1) \
            IE1 = (f); \
        else if (inum == INUM_TIMER1) \
            TF1 = (f); \
        else if (inum == INUM_UART0_RX) \
            RI0 = (f); \
        else if (inum == INUM_UART0_TX) \
            TI0 = (f); \
        else if (inum == INUM_TIMER2_H) \
            TF2H = (f); \
        else if (inum == INUM_TIMER2_L) \
            TF2L = (f); \
        else if (inum == INUM_SPI0_IF) \
            SPIF = (f); \
        else if (inum == INUM_SPI0_WCOL) \
            WCOL = (f); \
        else if (inum == INUM_SPI0_MODF) \
            MODF = (f); \
        else if (inum == INUM_SPI0_RXOVRN) \
            RXOVRN = (f); \
        else if (inum == INUM_SMB0) \
            SI = (f); \
        else if (inum == INUM_ADC0WINDOW) \
            AD0WINT = (f); \
        else if (inum == INUM_ADC0) \
            AD0INT = (f); \
        else if (inum == INUM_PCA0_IF) \
            CF = (f); \
        else if (inum == INUM_PCA0_CCF0) \
            CCF0 = (f); \
        else if (inum == INUM_PCA0_CCF1) \
            CCF1 = (f); \
        else if (inum == INUM_PCA0_CCF2) \
            CCF2 = (f); \
        else if (inum == INUM_PCA0_CCF3) \
            CCF3 = (f); \
        else if (inum == INUM_PCA0_CCF4) \
            CCF4 = (f); \
        else if (inum == INUM_ACOMP0_F) \
            (f ? (CPT0CN |= BM_CP0FIF) : (CPT0CN &= (~BM_CP0FIF))); \
        else if (inum == INUM_ACOMP0_R) \
            (f ? (CPT0CN |= BM_CP0RIF) : (CPT0CN &= (~BM_CP0RIF))); \
        else if (inum == INUM_ACOMP1_F) \
            (f ? (CPT1CN |= BM_CP1FIF) : (CPT1CN &= (~BM_CP1FIF))); \
        else if (inum == INUM_ACOMP1_R) \
            (f ? (CPT1CN |= BM_CP1RIF) : (CPT1CN &= (~BM_CP1RIF))); \
        else if (inum == INUM_TIMER3_H) \
            (f ? (TMR3CN |= BM_TF3H) : (TMR3CN &= (~BM_TF3H))); \
        else if (inum == INUM_TIMER3_L) \
            (f ? (TMR3CN |= BM_TF3L) : (TMR3CN &= (~BM_TF3L))); \
    } while (0)

//where f is one of:
#define INT_SET     1
#define INT_CLR     0
//-------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************               EXTERNAL INTERRUPTS                 **************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains macros for setting up GDO0 and GDO2 interrupt from CCxxx0. 
//-------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
//  SETUP_GDO0_INT(trigger, polarity)
//
//  DESCRIPTION:
//      This macro is setting up the GDO0 interrupt from CCxx00. The interrupt is on P0.6 and is 
//      assign to external interrupt0. The macro enables external interrupt0.
//
//  ARGUMENTS:
//      trigger
//          EDGE    (interrupt is edge sensitive)
//          LEVEL   (interrupt is level sensitive)
//      polarity
//          HIGH    (input is active high)
//          LOW     (input is active low)
//------------------------------------------------------------------------------------------------------
#define SETUP_GDO0_INT(trigger, polarity) \
    do { \
        IT0 = trigger; \
        IT01CF = ((IT01CF & 0xF0) | ((polarity << 3) | 0x06)); \
        INT_ENABLE(INUM_EXTERNAL0, INT_ON); \
    } while (0)
//------------------------------------------------------------------------------------------------------    


 
    
//------------------------------------------------------------------------------------------------------
//  SETUP_GDO2_INT(trigger, polarity)
//
//  DESCRIPTION:
//      This macro is setting up the GDO2 interrupt from CCxx00. The interrupt is on P0.7 and is 
//      assign to external interrupt1. The macro enables external interrupt1.
//
//  ARGUMENTS:
//      trigger
//          EDGE    (interrupt is edge sensitive)
//          LEVEL   (interrupt is level sensitive)
//      polarity
//          HIGH    (input is active high)
//          LOW     (input is active low)
//------------------------------------------------------------------------------------------------------
#define SETUP_GDO2_INT(trigger, polarity) \
    do { \
        IT1 = trigger; \
        IT01CF = ((IT01CF & 0x0F) | ((polarity << 7) | 0x70)); \
        INT_ENABLE(INUM_EXTERNAL1, INT_ON); \
    } while (0)
//------------------------------------------------------------------------------------------------------

// where trigger is one of:
#define LEVEL   0
#define EDGE    1

// where polarity is one of:
#define LOW     0
#define HIGH    1
//------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                 UART SERIAL PORT                 ***************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains useful macros for configuring the UART and for TX and RX using the UART
//      UART_SETUP(baudRate, options)
//      UART_TX_ENABLE()
//      UART_RX_ENABLE()
//      UART_TX_WAIT()
//      UART_RX_WAIT()
//      UART_TX(x)
//      UART_RX(x)
//      UART_WAIT_AND_SEND(x)
//      UART_WAIT_AND_RECEIVE(x)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Options for UART configuration
#define DEFAULT_MODE        0x00    // 8-bit Uart with variable baudrate, logic level of stop bit is 
                                    // ignored. User must poll the RI_0 and TI_0 flag to determine when a 
                                    // byte arrives or when the TX buffer is empty. The user must manually
                                    // clear RI0 and TI0 if not using the printf() / scanf() functions 
                                    // which do this automatically. 
#define UART_ISR            0x01    // Use interrupt service routines to signal that a byte has arrived or
                                    // that the TX buffer is empty. The user must manually clear RI_0 and
                                    // TI_0 in the corresponding ISRs.
//-------------------------------------------------------------------------------------------------------

                                   


//-------------------------------------------------------------------------------------------------------
// Defines used for the _baudrate_ variable
#define UART_BAUDRATE_4800      0
#define UART_BAUDRATE_9600      1            
#define UART_BAUDRATE_19200     2
#define UART_BAUDRATE_38400     3
#define UART_BAUDRATE_57600     4
#define UART_BAUDRATE_115200    5
//-------------------------------------------------------------------------------------------------------

 


//------------------------------------------------------------------------------------------------------
//  void halUartSetup(UINT16 baudRate, UINT8 options)
//
//  DESCRIPTION:
//      Function which implements all the initialization necessary to establish a simple serial link. 
//      Timer1 is used as a baudrate generator and is initialized according to _baudRate_. Timer 1 
//      is enabled and configures for Mode 2; 8-bit counter/timer with auto-reload. The UART 
//      is configured according to _options_.
//      The system clock needs to be configured to run at 24 MHz. 
//
//  ARGUMENTS:
//      UINT16 baudRate
//          UART_BAUDRATE_4800
//          UART_BAUDRATE_9600
//          UART_BAUDRATE_19200
//          UART_BAUDRATE_38400
//          UART_BAUDRATE_57600
//          UART_BAUDRATE_115200
//      UINT16 clkFreq
//          Device clock frequency in kHz
//      UINT8 options
//          DEFAULT_MODE           
//          UART_ISR
//------------------------------------------------------------------------------------------------------
void halUartSetup(UINT8 baudRate, UINT8 options);




//-------------------------------------------------------------------------------------------------------
// Macros which are helpful when transmitting and receiving data over the serial interface.
//
// Example of usage:
//
//      UART_TX_ENABLE();
//      UART_TX(data);
//
//      for (i = 1; i < len; i++) {
//          UART_WAIT_AND_SEND(pData[i]);
//      }
//
//      UART_RX_ENABLE();
//      len = UART_RX();
//
//      while (len-- > 0) {
//          UART_WAIT_AND_RECEIVE(data[i++]);
//      }

#define UART_TX_ENABLE()            do { REN0 = 0; } while (0)
#define UART_RX_ENABLE()            do { REN0 = 1; } while (0)

#define UART_TX_WAIT()              do {while (!INT_GETFLAG(INUM_UART0_TX)); } while (0)
#define UART_RX_WAIT()              do {while (!INT_GETFLAG(INUM_UART0_RX)); } while (0) 

#define UART_TX(x)                  do { SBUF0 = (x); } while (0)
#define UART_RX(x)                  do { (x) = SBUF0; } while (0)

#define UART_WAIT_AND_SEND(x) \
    do { \
        UART_TX_WAIT(); \
        INT_SETFLAG(INUM_UART0_TX, INT_CLR); \
        UART_TX(x); \
    } while (0)

#define UART_WAIT_AND_RECEIVE(x) \
    do { \
        UART_RX_WAIT(); \
        UART_RX(x); \
        INT_SETFLAG(INUM_UART0_RX, INT_CLR); \  
} while (0)
//-------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 *******************************              SPI functions/macros        ******************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains useful macros and functions for setting up the SPI and for reading/writing
// CC2500 registers
//      SPI_INIT(freq)
//      SPI_ENABLE()
//      SPI_DISABLE()
//      SPI_WAIT()
//      void halSpiStrobe(BYTE addr)
//      void halSpiWriteReg(BYTE addr, BYTE value)
//      BYTE halSpiReadReg(BYTE addr)
//      void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//      halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//      RESET_CCxxx0()
//      POWER_UP_CCxxx0()
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Definitions to support burst/single access:
#define WRITE_BURST     0x40
#define READ_SINGLE     0x80
#define READ_BURST      0xC0
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// SPI enable/disable macros:
#define SPI_ENABLE()        (SPI0CN |= BM_SPIEN)
#define SPI_DISABLE()       (SPI0CN &= ~BM_SPIEN)   
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Initialization
// Enble SPI (4-wire Single Master Mode, data centered on first edge of SCK period.
// SCK low in Idle State
#define SPI_INIT(freq) \
    do { \
        SPI0CFG = BM_MSTEN; \
        SPI0CN = BM_NSSMD1; \
        SPI0CKR = freq; \
        SPI_ENABLE(); \
    } while (0)

// where freq is one of:
#define SCLK_6_MHZ      1
#define SCLK_4_MHZ      2
#define SCLK_3_MHZ      3
#define SCLK_2_4_MHZ    4
#define SCLK_2_MHZ      5
#define SCLK_1_5_MHZ    7
#define SCLK_1_2_MHZ    9
#define SCLK_1_MHZ      11
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro used for communication data polling and wait on the SPI bus 
#define SPI_WAIT() \
    do { \
        while (!INT_GETFLAG(INUM_SPI0_IF)); \
        INT_SETFLAG(INUM_SPI0_IF, INT_CLR); \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void halSpiStrobe(BYTE strobe)
//
//  DESCRIPTION:
//      Function for writing a strobe command to the CCxxx0
//
//  ARGUMENTS:
//      BYTE strobe
//          Strobe command
//-------------------------------------------------------------------------------------------------------
void halSpiStrobe(BYTE strobe);




//-------------------------------------------------------------------------------------------------------
//  BYTE halSpiReadStatus(BYTE addr)
//
//  DESCRIPTION:
//      This function reads a CCxxx0 status register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 status register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 status register.
//-------------------------------------------------------------------------------------------------------
BYTE halSpiReadStatus(BYTE addr);




//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteReg(BYTE addr, BYTE value)
//
//  DESCRIPTION:
//      Function for writing to a single CCxxx0 register
//
//  ARGUMENTS:
//      BYTE addr
//          Address of a specific CCxxx0 register to accessed.
//      BYTE value
//          Value to be written to the specified CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
void halSpiWriteReg(BYTE addr, BYTE value);




//-------------------------------------------------------------------------------------------------------
//  BYTE halSpiReadReg(BYTE addr)
//
//  DESCRIPTION:
//      This function gets the value of a single specified CCxxx0 register.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the CCxxx0 register to be accessed.
//
//  RETURN VALUE:
//      BYTE
//          Value of the accessed CCxxx0 register.
//-------------------------------------------------------------------------------------------------------
BYTE halSpiReadReg(BYTE addr);




//-------------------------------------------------------------------------------------------------------
//  void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function writes to multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Array of bytes to be written into a corresponding range of
//          CCxx00 registers, starting by the address specified in _addr_.
//      BYTE count
//          Number of bytes to be written to the subsequent CCxxx0 registers.   
//-------------------------------------------------------------------------------------------------------
void halSpiWriteBurstReg(BYTE addr, BYTE *buffer, BYTE count);




//-------------------------------------------------------------------------------------------------------
//  void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count)
//
//  DESCRIPTION:
//      This function reads multiple CCxxx0 register, using SPI burst access.
//
//  ARGUMENTS:
//      BYTE addr
//          Address of the first CCxxx0 register to be accessed.
//      BYTE *buffer
//          Pointer to a byte array which stores the values read from a
//          corresponding range of CCxxx0 registers.
//      BYTE count
//          Number of bytes to be read from the subsequent CCxxx0 registers.
//-------------------------------------------------------------------------------------------------------
void halSpiReadBurstReg(BYTE addr, BYTE *buffer, BYTE count);




//-------------------------------------------------------------------------------------------------------
// Macro to reset the CCxxx0 and wait for it to be ready
#define RESET_CCxxx0() \
    do { \
        NSSMD0 = 0; \
        while (P0_1); \
        SPI0DAT = CCxxx0_SRES; \
        SPI_WAIT(); \
        NSSMD0 = 1; \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro to reset the CCxxx0 after power_on and wait for it to be ready
// IMPORTANT NOTICE:
// The file Wait.c must be included if this macro shall be used
// The file is located under: ..\Lib\Chipcon\Hal\CCxx00
//
//                 min 40 us
//             <----------------------->
// CSn      |--|  |--------------------|          |-----------
//          |  |  |                    |          |
//              --                      ----------
//
// MISO                                       |---------------
//          - - - - - - - - - - - - - - - -|  |         
//                                          --          
//               Unknown / don't care
//
// MOSI     - - - - - - - - - - - - - - - ---------- - - - - - 
//                                         | SRES |
//          - - - - - - - - - - - - - - - ---------- - - - - -                    
//
#define POWER_UP_RESET_CCxxx0() \
    do { \
        NSSMD0 = 1; \
        halWait(1); \
        NSSMD0 = 0; \
        halWait(1); \
        NSSMD0 = 1; \
        halWait(41); \
        RESET_CCxxx0(); \
    } while (0)
//-------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************              Timer macros/functions             ****************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains macros and functions for setting up the 4 different timers available
//      TIMER0_RUN(x)
//      TIMER1_RUN(x)
//      TIMER2_RUN(x)
//      TIMER3_RUN(x)
//      SET_RELOAD_VALUE_TIMER0(period_us, clock_kHz)
//      SET_RELOAD_VALUE_TIMER1(period_us, clock_kHz)
//      SET_RELOAD_VALUE_TIMER2_8BIT(periodH_us, periodL_us, clock_kHzH, clock_kHzL)
//      SET_RELOAD_VALUE_TIMER3_8BIT(periodH_us, periodL_us, clock_kHzH, clock_kHzL)
//      SET_RELOAD_VALUE_TIMER2_16BIT(period_us, clock_kHz)
//      SET_RELOAD_VALUE_TIMER3_16BIT(period_us, clock_kHz)
//      void halSetupTimer01(UINt8 timer01, UINT8 clkSource, UINT8 mode, BOOL timerInt)
//      void halSetupTimer23(UINT8 timer23, UINT8 clkSourceH, UINT8 clkSourceL, UINT8 mode, BOOL timerInt)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Defines used for variable _clkSource_, _clkSourceL_, and _clkSourceH_
#define SYSCLK_DIV_12   0
#define SYSCLK_DIV_4    1
#define SYSCLK_DIV_48   2
#define EXTCLK_DIV_8    3
#define SYSCLK_DIV_1    4

// Define used for variable _timer01_ 
#define TIMER_0     0
#define TIMER_1     1

// Define used for variable _timer23_ 
#define TIMER_2     2
#define TIMER_3     3

// Defines used for variable _mode_
#define MODE_0      0       // Timer0, timer1, timer2, and timer3
#define MODE_1      1       // Timer0, timer1, timer2, and timer3
#define MODE_2      2       // Timer0 and timer1
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Macro for stopping and starting the timers.
#define TIMER0_RUN(x)    (TR0 = !!(x))
#define TIMER1_RUN(x)    (TR1 = !!(x))
#define TIMER2_RUN(x)    (TR2 = !!(x))
#define TIMER3_RUN(x)    (x ? (TMR3CN |= BM_TR3) : (TMR3CN &= (~BM_TR3)))
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  SET_RELOAD_VALUE_TIMERx(period_us, clock_kHz)
//
//  DESCRIPTION:
//      Macros used to calculate the reload value and update the reload registers.
//      x = 0 or 1
//
//      Below is a table showing what periods are possible for the different clock sources
//
//      Clock Source:   |   Freq:   |   Min. period |   Max period
//      ----------------------------------------------------------
//      SYSCLK_DIV_1    |   24 MHz  |   41.7 ns     |   10.6 us      
//      SYSCLK_DIV_4    |   6 MHz   |   166.7 ns    |   42.6 us
//      SYSCLK_DIV_12   |   2 MHz   |   500 ns      |   128 us
//      SYSCLK_DIV_48   |   0.5 MHz |   2 us        |   512 us
//      ----------------------------------------------------------
//
//      Example of usage:
//
//      SET_RELOAD_VALUE_TIMER0(0.0417, 24000);
//  
//      halSetupTimer01(TIMER_0, SYSCLK_DIV_1, MODE_2, INT_ON);
//
//      IMPORTANT NOTICE:
//      Constants should be used as arguments when using the SET_RELOAD_VALUE_TIMERx macro in order
//      to reduce code size.
//      
//  ARGUMENTS:
//      period_us
//          The period between interrupts. The period must be given in us.
//
//      clock_kHz
//          The frequency of the clock source (in kHz)
//-------------------------------------------------------------------------------------------------------
// This macro is only for internal use in this library file
#define SET_RELOAD_VALUE_TIMER01(timer01, period_us, clock_kHz) \
    do { \
        UINT8 count; \
        count = (UINT8)(((float)period_us * (float)clock_kHz) / 1000); \
        TH##timer01 = 256 - count; \
        TL##timer01 = TH##timer01; \
    } while (0)
//-------------------------------------------------------------------------------------------------------
// These macros are available for the user
#define SET_RELOAD_VALUE_TIMER0(period_us, clock_kHz)  SET_RELOAD_VALUE_TIMER01(0, period_us, clock_kHz)
#define SET_RELOAD_VALUE_TIMER1(period_us, clock_kHz)  SET_RELOAD_VALUE_TIMER01(1, period_us, clock_kHz)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  SET_RELOAD_VALUE_TIMERx_8BIT(periodH_us, periodL_us, clock_kHzH, clock_kHzL)
//
//  DESCRIPTION:
//      Macros used to calculate the reload value and update the reload registers.
//      x = 2 or 3
//
//      Below is a table showing what periods are possible for the different clock sources
//
//      Clock Source:   |   Freq:   |   Min. period |   Max period
//      ----------------------------------------------------------
//      SYSCLK_DIV_1    |   24 MHz  |   41.7 ns     |   10.6 us      
//      SYSCLK_DIV_12   |   2 MHz   |   500 ns      |   128 us
//      ----------------------------------------------------------
//
//      Example of usage:
//
//      SET_RELOAD_VALUE_TIMER2_8BIT(3, 60, 24000, 2000);
//  
//      halSetupTimer23(TIMER_2, SYSCLK_DIV_1, SYSCLK_DIV_12, MODE_1, INT_ON);
//
//      IMPORTANT NOTICE:
//      Constants should be used as arguments when using the SET_RELOAD_VALUE_TIMERx_8BIT macro in order
//      to reduce code size.
//      
//  ARGUMENTS:
//      periodH_us
//          The period between interrupts (high byte overflow). The period must be given in us.
//
//      periodL_us
//          The period between interrupts (low byte overflow). The period must be given in us.
//
//      clock_kHzH
//          The frequency of the clock source for timer TMRxH (in kHz)
//
//      clock_kHzL
//          The frequency of the clock source for timer TMRxL (in kHz)
//-------------------------------------------------------------------------------------------------------
// This macro is only for internal use in this library file
#define SET_RELOAD_VALUE_TIMER23_8BIT(timer23, periodH_us, periodL_us, clock_kHzH, clock_kHzL) \
    do { \
        UINT8 countH; \
        UINT8 countL; \
        countH = (UINT8)(((float)periodH_us * (float)clock_kHzH) / 1000); \
        countL = (UINT8)(((float)periodL_us * (float)clock_kHzL) / 1000); \          
        TMR##timer23##RLH = 256 - countH; \
        TMR##timer23##H = TMR##timer23##RLH; \
        TMR##timer23##RLL = 256 - countL; \
        TMR##timer23##L = TMR##timer23##RLL; \
    } while (0)
//-------------------------------------------------------------------------------------------------------
// These macros are available for the user
#define SET_RELOAD_VALUE_TIMER2_8BIT(periodH_us, periodL_us, clock_kHzH, clock_kHzL) \
    do { \
        SET_RELOAD_VALUE_TIMER23_8BIT(2, periodH_us, periodL_us, clock_kHzH, clock_kHzL); \
    } while (0)

#define SET_RELOAD_VALUE_TIMER3_8BIT(periodH_us, periodL_us, clock_kHzH, clock_kHzL) \
    do { \
        SET_RELOAD_VALUE_TIMER23_8BIT(3, periodH_us, periodL_us, clock_kHzH, clock_kHzL); \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  SET_RELOAD_VALUE_TIMERx_16BIT(period_us, clock_kHz)
//
//  DESCRIPTION:
//      Macros used to calculate the reload value and update the reload registers.
//      x = 2 or 3
//
//      Below is a table showing what periods are possible for the different clock sources
//
//      Clock Source:   |   Freq:   |   Min. period |   Max period
//      ----------------------------------------------------------
//      SYSCLK_DIV_1    |   24 MHz  |   41.7 ns     |   2.73 ms      
//      SYSCLK_DIV_12   |   2 MHz   |   500 ns      |   32.75 ms
//      ----------------------------------------------------------
//
//      Example of usage:
//
//      SET_RELOAD_VALUE_TIMER2_16BIT(100, 24000);
//  
//      halSetupTimer23(TIMER_2, NULL, SYSCLK_DIV_1, MODE_0, INT_ON);
//
//      IMPORTANT NOTICE:
//      Constants should be used as arguments when using the SET_RELOAD_VALUE_TIMERx_16BIT macro in order
//      to reduce code size.
//      
//  ARGUMENTS:
//      period_us
//          The period between interrupts. The period must be given in us.
//
//      clock_kHzH
//          The frequency of the clock source (in kHz)
//-------------------------------------------------------------------------------------------------------
// This macro is only for internal use in this library file
#define SET_RELOAD_VALUE_TIMER23_16BIT(timer23, period_us, clock_kHz) \
    do { \
        UINT16 count; \
        UINT16 reloadValue; \
        count = (UINT16)(((float)period_us * (float)clock_kHz) / 1000); \
        reloadValue = (UINT16)(65536 - count); \
        TMR##timer23##RLL = reloadValue; \
        TMR##timer23##RLH = reloadValue >> 8; \
        TMR##timer23##L = TMR##timer23##RLL; \
        TMR##timer23##H = TMR##timer23##RLH; \
    } while (0)
//-------------------------------------------------------------------------------------------------------
// These macros are available for the user
#define SET_RELOAD_VALUE_TIMER2_16BIT(period_us, clock_kHz) \
    do { \
        SET_RELOAD_VALUE_TIMER23_16BIT(2, period_us, clock_kHz); \
    } while (0)

#define SET_RELOAD_VALUE_TIMER3_16BIT(period_us, clock_kHz) \
    do { \
        SET_RELOAD_VALUE_TIMER23_16BIT(3, period_us, clock_kHz); \
    } while (0)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
#define INTERRUPT       TRUE
#define NO_INTERRUPT    FALSE
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// void halSetupTimer01(UINT8 timer01, UINT8 clkSource, UINT8 mode, BOOL timerInt)
//
//  DESCRIPTION:
//      Function used to set up timer 0 or timer 1. Be aware that Timer 0 and Timer 1 share the same
//      prescaler. The function enables the given timer. Allowing the timer to be controlled by the 
//      external input signal can be done manually by setting GATE0 or GATE1 to ‘1'.
//      If mode 2 i choosen, the SET_RELOAD_VALUE_TIMER0 or  SET_RELOAD_VALUE_TIMER1 macro has to
//      be called before this function.
//
//  ARGUMENTS:
//      UINT8 timer01
//          TIMER_0
//          TIMER_1
//
//      UINT8 clkSource 
//          SYSCLK_DIV_12
//          SYSCLK_DIV_4
//          SYSCLK_DIV_48
//          EXTCLK_DIV_8
//          SYSCLK_DIV_1
//
//      UINT8 mode
//          MODE_0  (13-bit counter/timer)
//          MODE_1  (16-bit counter/timer)
//          MODE_2  (8-bit counter/timer with auto-reload)
//
//          IMPORTANT NOTICE:
//          Timer 0 has a fourth mode (Mode 3). This mode has some restictions and needs to be  
//          configured manually 
//      
//      BOOL timerInt
//          TRUE: Enable timer interrupt
//          False: Do nothing (does not disable timer interrupt if it has been enabled outside this
//          function).
//-------------------------------------------------------------------------------------------------------
void halSetupTimer01(UINT8 timer01, UINT8 clkSource, UINT8 mode, BOOL timerInt);




//-------------------------------------------------------------------------------------------------------
//  void halSetupTimer23(UINT8 timerOption, clkSourceH, UINT8 clkSourceL, UINT8 mode, BOOL timerInt)
//
//  DESCRIPTION:
//      Function used to set up timer 2 or timer 3. SET_RELOAD_VALUE_TIMERx_16BIT(period, clock_kHz),
//      used for mode 0, or SET_RELOAD_VALUE_TIMERx_8BIT(periodH, periodL, clock_kHzH, clock_kHzL),
//      used for mode 1,  should first be called (x = 2 or 3). The function enables the given timer. 
//      This function only support mode 0 and mode 1.
//
//  ARGUMENTS:
//      UINT8 timer23
//          TIMER_2
//          TIMER_3
//
//      UINT8 clkSourceH, clkSourceL 
//          SYSCLK_DIV_12
//          EXTCLK_DIV_8
//          SYSCLK_DIV_1
//
//          IMPORTANT NOTICE:
//          If mode 0 is choosen, clkSourceL should be initialized and clkSourceH is DON'T CARE
//          In mode 2, both clock sources must be given a value from the list above
//
//      UINT8 mode
//          MODE_0  (16-bit timer with auto-reload)
//          MODE_1  (8-bit timers with auto-reload)
//      
//      BOOL timerInt
//          TRUE: Enable timer interrupt
//          False: Do nothing (does not disable timer interrupt if it has been enabled outside this
//          function). 
//          For mode 0, only TMRxH will generate interrupt.         
//          For mode 1, both TMRxH AND TMRxL will generate interrupt. If only TMRxH shall generate 
//          interrupt, TFxLEN must be cleared manually after this function has been called 
//          IMPORTANT NOTICE:
//          TF2H and TF2L must be cleared by software in the ISR
//          Software must also check what caused the interrupt (high byte or low byte overflow)
//-------------------------------------------------------------------------------------------------------
void halSetupTimer23(UINT8 timer23, UINT8 clkSourceH, UINT8 clkSourceL, UINT8 mode, BOOL timerInt);





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                 ADC macros/functions            ****************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains macros used to enable/disable  and sample the ADC.
// Examples on how to initialize the ADC converter can be found in GetJoystickPosition.c and
// Read Potensiometer.c
//      ADC_ENABLE()
//      ADC_DISABLE()
//      ADC_SAMPLE()
//------------------------------------------------------------------------------------------------------- 
 
 
 
 
//------------------------------------------------------------------------------------------------------- 
#define ADC_ENABLE()    (AD0EN = 1)
#define ADC_DISABLE()   (AD0EN = 0)

// This macro clears the ADC0 Conversion Complete Interrupt Flag, initiates ADC0 conversion and
// waits for the conversion to complete
#define ADC_SAMPLE() \
    do { \
        AD0INT = 0; \
        AD0BUSY |= 0x10; \
        while (!AD0INT); \
    } while (0)
//------------------------------------------------------------------------------------------------------- 





/*******************************************************************************************************
 *******************************************************************************************************
 **************************        RF communication functions/macros        ****************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains functions used to write rf settings to CCxxx0, and for sending and 
// receiving a packet.
//      void RfWriteRfSettings(RF_SETTINGS *pRfSettings);
//      void halRfSendPacket(BYTE *txBuffer, UINT8 size);
//      BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//      void halRfSendPacketSerial(BYTE *txBuffer, UINT8 size, UINT8 startOfPayload, BOOL crcEnable);
//      BOOL halRfReceivePacketSerial
//      (BYTE *rxBuffer, UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0, UINT8 fixedLength, BOOL crcEnable);
//      BOOL halRfSendPacketLockDetect(BYTE *txBuffer, UINT8 size, BOOL *pLock);
//      BOOL halRfReceivePacketLockDetect(BYTE *rxBuffer, UINT8 *length, BOOL *pLock);
//-------------------------------------------------------------------------------------------------------



 
//-------------------------------------------------------------------------------------------------------
// RF_SETTINGS is a data structure which contains all relevant CCxxx0 registers
typedef struct S_RF_SETTINGS{
    BYTE FSCTRL1;   // Frequency synthesizer control.
    BYTE FSCTRL0;   // Frequency synthesizer control.
    BYTE FREQ2;     // Frequency control word, high byte.
    BYTE FREQ1;     // Frequency control word, middle byte.
    BYTE FREQ0;     // Frequency control word, low byte.
    BYTE MDMCFG4;   // Modem configuration.
    BYTE MDMCFG3;   // Modem configuration.
    BYTE MDMCFG2;   // Modem configuration.
    BYTE MDMCFG1;   // Modem configuration.
    BYTE MDMCFG0;   // Modem configuration.
    BYTE CHANNR;    // Channel number.
    BYTE DEVIATN;   // Modem deviation setting (when FSK modulation is enabled).
    BYTE FREND1;    // Front end RX configuration.
    BYTE FREND0;    // Front end RX configuration.
    BYTE MCSM0;     // Main Radio Control State Machine configuration.
    BYTE FOCCFG;    // Frequency Offset Compensation Configuration.
    BYTE BSCFG;     // Bit synchronization Configuration.
    BYTE AGCCTRL2;  // AGC control.
	BYTE AGCCTRL1;  // AGC control.
    BYTE AGCCTRL0;  // AGC control.
    BYTE FSCAL3;    // Frequency synthesizer calibration.
    BYTE FSCAL2;    // Frequency synthesizer calibration.
	BYTE FSCAL1;    // Frequency synthesizer calibration.
    BYTE FSCAL0;    // Frequency synthesizer calibration.
    BYTE FSTEST;    // Frequency synthesizer calibration control
    BYTE TEST2;     // Various test settings.
    BYTE TEST1;     // Various test settings.
    BYTE TEST0;     // Various test settings.
    BYTE FIFOTHR;   // RXFIFO and TXFIFO thresholds.
    BYTE IOCFG2;    // GDO2 output pin configuration
    BYTE IOCFG0;    // GDO0 output pin configuration
    BYTE PKTCTRL1;  // Packet automation control.
    BYTE PKTCTRL0;  // Packet automation control.
    BYTE ADDR;      // Device address.
    BYTE PKTLEN;    // Packet length.
} RF_SETTINGS;

    





//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void RfWriteRfSettings(RF_SETTINGS *pRfSettings)
//
//  DESCRIPTION:
//      This function is used to configure the CC2500 based on a given rf setting
//
//  ARGUMENTS:
//      RF_SETTINGS *pRfSettings
//          Pointer to a struct containing rf register settings
//-------------------------------------------------------------------------------------------------------
void halRfWriteRfSettings(RF_SETTINGS *pRfSettings);




//-------------------------------------------------------------------------------------------------------
//  void halRfSendPacket(BYTE *txBuffer, UINT8 size)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to 63 bytes.
//      To use this function, GD02 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG2, 0x06);
//      The function implements polling of GDO2. First it waits for GD02 to be set and then it waits
//      for it to be cleared.  
//      
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containg the data that are going to be transmitted
//
//      UINT8 size
//          The size of the txBuffer
//-------------------------------------------------------------------------------------------------------
void halRfSendPacket(BYTE *txBuffer, UINT8 size);




//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length)
//
//  DESCRIPTION: 
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD02 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG2, 0x06);
//      The function implements polling of GDO2. First it waits for GD02 to be set and then it waits
//      for it to be cleared.  
//  
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//          
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacket(BYTE *rxBuffer, UINT8 *length);




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
void halRfSendPacketSerial(BYTE *txBuffer, UINT8 size, UINT8 startOfPayload, BOOL crcEnable);




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
(BYTE *rxBuffer, UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0, UINT8 fixedLength, BOOL crcEnable);




//-------------------------------------------------------------------------------------------------------
//  BOOL halRfSendPacketLockDetect(BYTE *txBuffer, UINT8 size, BOOL *pLock)
//
//  DESCRIPTION:
//      This function can be used to transmit a packet with packet length up to 63 bytes.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      GD02 must be configured to be asserted when lock is detected
//      => halSpiWriteReg(CCxxx0_IOCFG2, 0x0A);
//      If the PLL is not locked within ~920 us, the radio is forced back in IDLE state and no packet is
//      being sent.If the PLL is locked within 920 us, TX mode is entered.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.  
//      
//  ARGUMENTS:
//      BYTE *txBuffer
//          Pointer to a buffer containing the data that are going to be transmitted
//
//      UINT8 size
//          The size of the txBuffer
//
//      BOOL *pLock
//          Pointer to a variable that is set in an ISR when the Lock Detector is asserted.
//
//  RETURN VALUE:
//      BOOL
//          TRUE; A packet has been sent
//          FALSE: Radio is forced to IDLE state due to lock failure. No packet is being sent.
//-------------------------------------------------------------------------------------------------------
BOOL halRfSendPacketLockDetect(BYTE *txBuffer, UINT8 size, BOOL *pLock);




//-------------------------------------------------------------------------------------------------------
//  BOOL halRfReceivePacketLockDetect(BYTE *rxBuffer, UINT8 *length, *pLock)
//
//  DESCRIPTION: 
//      This function can be used to receive a packet of variable packet length (first byte in the packet
//      must be the length byte). The packet length should not exceed the RX FIFO size.
//      To use this function, GD00 must be configured to be asserted when sync word is sent and 
//      de-asserted at the end of the packet => halSpiWriteReg(CCxxx0_IOCFG0, 0x06);
//      GD02 must be configured to be asserted when lock is detected
//      => halSpiWriteReg(CCxxx0_IOCFG2, 0x0A);
//      Also, APPEND_STATUS in the PKTCTRL1 register must be enabled. If the PLL is not locked within
//      ~920 us the radio is forced to IDLE state and the RX FIFO is being flushed.
//      The function implements polling of GDO0. First it waits for GD00 to be set and then it waits
//      for it to be cleared.
//      After the GDO0 pin has been de-asserted, the RXBYTES register is read to make sure that there
//      are bytes in the FIFO. This is because the GDO signal will indicate sync received even if the
//      FIFO is flushed due to address filtering, CRC filtering, or packet length filtering. 
//  
//  ARGUMENTS:
//      BYTE *rxBuffer
//          Pointer to the buffer where the incoming data should be stored
//
//      UINT8 *length
//          Pointer to a variable containing the size of the buffer where the incoming data should be
//          stored. After this function returns, that variable holds the packet length.
//
//      BOOL *pLock
//          Pointer to a variable that is set in an ISR when the Lock Detector is asserted.
//          
//  RETURN VALUE:
//      BOOL
//          TRUE:   CRC OK
//          FALSE:  CRC NOT OK, 
//                      or 
//                  no packet was put in the RX FIFO due to filtering
//                      or
//                  the PLL did not lock 
//-------------------------------------------------------------------------------------------------------
BOOL halRfReceivePacketLockDetect(BYTE *rxBuffer, UINT8 *length, BOOL *pLock);




/*******************************************************************************************************
 *******************************************************************************************************
 *************************     Oscillator control macros/functions       ***************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains a macro for initializing the internal oscillator, the system clock and the
// 4x Clock Multiplier
//      CLOCK_INIT()    
//-------------------------------------------------------------------------------------------------------

    
    

//-------------------------------------------------------------------------------------------------------
#define INT_OSC             0x00
#define FOUR_X_CLK_MULT     0x02
#define INT_OSC_DIV_1       0x83
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
// Select the Internal Oscillator as Multiplier input source and disable the watchdog timer
// SYSCLK = 4X Clock Multiplier / 2     
#define CLOCK_INIT() \
    do { \
        UINT8 i; \
        PCA0MD &= ~0x40; \
        CLKMUL = INT_OSC; \
        CLKMUL |= 0x80; \
        for (i = 0; i < 20; i++); \
        CLKMUL |= 0xC0; \
        while (!(CLKMUL & BM_MULRDY)); \
        CLKSEL |= FOUR_X_CLK_MULT; \
        OSCICN = INT_OSC_DIV_1; \
    } while (0) 
//-------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************              Utility functions          ************************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains a function that implements a wait for [timeout] microseconds.
//      void halWait(UINT16 timeout)
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  DESCRIPTION:
//      Runs an idle loop for [timeout] microseconds.
//-------------------------------------------------------------------------------------------------------
void halWait(UINT16 timeout);




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: halsrf04.h,v $
 * Revision 1.16  2007/11/23 09:11:53  a0190596
 * FIFOTHR added to the RF_SETTINGS data structure
 *
 * Revision 1.15  2006/10/05 13:48:30  a0190596
 * no message
 *
 * Revision 1.14  2006/05/03 12:00:51  a0190596
 * while (P0_1);  removed after SPI_WAIT(); in the
 * RESET_CCxxx0() macro
 *
 * Revision 1.13  2006/04/03 06:19:26  a0190596
 * else if inum == INUM_ACOMP0) \
 * changed to
 * else if (inum == INUM_ACOMP0) " halsrf04.h
 *
 * Revision 1.12  2006/03/17 09:59:54  a0190596
 * Two new functions included:
 *
 * BOOL halRfSendPacketLockDetect(BYTE *txBuffer, UINT8 size, BOOL *pLock);
 * BOOL halRfReceivePacketLockDetect(BYTE *rxBuffer, UINT8 *length, BOOL *pLock);
 *
 * Revision 1.9  2005/11/09 10:04:54  sna
 * Added halRfReceivePacketSerial and halRfSendPacketSerial
 *
 * Revision 1.8  2005/10/25 09:48:53  sna
 * Added AGCCTRL1 and FSCAL1 to the RF_SETTINGS struct.
 *
 * Revision 1.7  2005/07/11 12:24:09  sna
 * while (P0_1); \ has been added to the RESET_CCxxx0() macro
 *
 * Revision 1.6  2005/06/14 11:04:31  sna
 * no message
 *
 * Revision 1.5  2005/05/26 15:19:02  sna
 * no message
 *
 * Revision 1.4  2005/05/26 15:12:46  sna
 * Updated the RF_SETTINGS struct
 *
 * Revision 1.3  2005/04/11 13:29:36  sna
 * Updated the RF_SETTINGS struct according to SmartRF Studio 6.0.4
 *
 * Revision 1.2  2005/02/07 14:34:55  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/




#endif//HALSRF04_H
