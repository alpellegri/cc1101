/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     Register definition file for CCxxx0 and c8051f320.                         *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     regssrf04.h                                                                *
 *      ***        ***                                                                                 *
 *       ************                                                                                  *
 *        **********                                                                                   *
 *                                                                                                     *
 *******************************************************************************************************
 * Compiler:             Keil C51 V7.50                                                                *
 * Target platform:      Chipcon CCxx00 (Silabs F320)                                                  *
 * Author:               SNA                                                                           *
 *******************************************************************************************************
 * Revision history:     See end of file                                                               *
 *******************************************************************************************************/
#ifndef REGSSRF04_H
#define REGSSRF04_H
#include "c8051f320.h" 




//------------------------------------------------------------------------------------------------------
// CC2500/CC1100 STROBE, CONTROL AND STATUS REGSITER
#define CCxxx0_IOCFG2       0x00        // GDO2 output pin configuration
#define CCxxx0_IOCFG1       0x01        // GDO1 output pin configuration
#define CCxxx0_IOCFG0       0x02        // GDO0 output pin configuration
#define CCxxx0_FIFOTHR      0x03        // RX FIFO and TX FIFO thresholds
#define CCxxx0_SYNC1        0x04        // Sync word, high byte
#define CCxxx0_SYNC0        0x05        // Sync word, low byte
#define CCxxx0_PKTLEN       0x06        // Packet length
#define CCxxx0_PKTCTRL1     0x07        // Packet automation control
#define CCxxx0_PKTCTRL0     0x08        // Packet automation control
#define CCxxx0_ADDR         0x09        // Device address
#define CCxxx0_CHANNR       0x0A        // Channel number
#define CCxxx0_FSCTRL1      0x0B        // Frequency synthesizer control
#define CCxxx0_FSCTRL0      0x0C        // Frequency synthesizer control
#define CCxxx0_FREQ2        0x0D        // Frequency control word, high byte
#define CCxxx0_FREQ1        0x0E        // Frequency control word, middle byte
#define CCxxx0_FREQ0        0x0F        // Frequency control word, low byte
#define CCxxx0_MDMCFG4      0x10        // Modem configuration
#define CCxxx0_MDMCFG3      0x11        // Modem configuration
#define CCxxx0_MDMCFG2      0x12        // Modem configuration
#define CCxxx0_MDMCFG1      0x13        // Modem configuration
#define CCxxx0_MDMCFG0      0x14        // Modem configuration
#define CCxxx0_DEVIATN      0x15        // Modem deviation setting
#define CCxxx0_MCSM2        0x16        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM1        0x17        // Main Radio Control State Machine configuration
#define CCxxx0_MCSM0        0x18        // Main Radio Control State Machine configuration
#define CCxxx0_FOCCFG       0x19        // Frequency Offset Compensation configuration
#define CCxxx0_BSCFG        0x1A        // Bit Synchronization configuration
#define CCxxx0_AGCCTRL2     0x1B        // AGC control
#define CCxxx0_AGCCTRL1     0x1C        // AGC control
#define CCxxx0_AGCCTRL0     0x1D        // AGC control
#define CCxxx0_WOREVT1      0x1E        // High byte Event 0 timeout
#define CCxxx0_WOREVT0      0x1F        // Low byte Event 0 timeout
#define CCxxx0_WORCTRL      0x20        // Wake On Radio control
#define CCxxx0_FREND1       0x21        // Front end RX configuration
#define CCxxx0_FREND0       0x22        // Front end TX configuration
#define CCxxx0_FSCAL3       0x23        // Frequency synthesizer calibration
#define CCxxx0_FSCAL2       0x24        // Frequency synthesizer calibration
#define CCxxx0_FSCAL1       0x25        // Frequency synthesizer calibration
#define CCxxx0_FSCAL0       0x26        // Frequency synthesizer calibration
#define CCxxx0_RCCTRL1      0x27        // RC oscillator configuration
#define CCxxx0_RCCTRL0      0x28        // RC oscillator configuration
#define CCxxx0_FSTEST       0x29        // Frequency synthesizer calibration control
#define CCxxx0_PTEST        0x2A        // Production test
#define CCxxx0_AGCTEST      0x2B        // AGC test
#define CCxxx0_TEST2        0x2C        // Various test settings
#define CCxxx0_TEST1        0x2D        // Various test settings
#define CCxxx0_TEST0        0x2E        // Various test settings

// Strobe commands
#define CCxxx0_SRES         0x30        // Reset chip.
#define CCxxx0_SFSTXON      0x31        // Enable and calibrate frequency synthesizer (if MCSM0.FS_AUTOCAL=1).
                                        // If in RX/TX: Go to a wait state where only the synthesizer is
                                        // running (for quick RX / TX turnaround).
#define CCxxx0_SXOFF        0x32        // Turn off crystal oscillator.
#define CCxxx0_SCAL         0x33        // Calibrate frequency synthesizer and turn it off
                                        // (enables quick start).
#define CCxxx0_SRX          0x34        // Enable RX. Perform calibration first if coming from IDLE and
                                        // MCSM0.FS_AUTOCAL=1.
#define CCxxx0_STX          0x35        // In IDLE state: Enable TX. Perform calibration first if
                                        // MCSM0.FS_AUTOCAL=1. If in RX state and CCA is enabled:
                                        // Only go to TX if channel is clear.
#define CCxxx0_SIDLE        0x36        // Exit RX / TX, turn off frequency synthesizer and exit
                                        // Wake-On-Radio mode if applicable.
#define CCxxx0_SAFC         0x37        // Perform AFC adjustment of the frequency synthesizer
#define CCxxx0_SWOR         0x38        // Start automatic RX polling sequence (Wake-on-Radio)
#define CCxxx0_SPWD         0x39        // Enter power down mode when CSn goes high.
#define CCxxx0_SFRX         0x3A        // Flush the RX FIFO buffer.
#define CCxxx0_SFTX         0x3B        // Flush the TX FIFO buffer.
#define CCxxx0_SWORRST      0x3C        // Reset real time clock.
#define CCxxx0_SNOP         0x3D        // No operation. May be used to pad strobe commands to two
                                        // bytes for simpler software.

#define CCxxx0_PARTNUM          0x30
#define CCxxx0_VERSION          0x31
#define CCxxx0_FREQEST          0x32
#define CCxxx0_LQI              0x33
#define CCxxx0_RSSI             0x34
#define CCxxx0_MARCSTATE        0x35
#define CCxxx0_WORTIME1         0x36
#define CCxxx0_WORTIME0         0x37
#define CCxxx0_PKTSTATUS        0x38
#define CCxxx0_VCO_VC_DAC       0x39
#define CCxxx0_TXBYTES          0x3A
#define CCxxx0_RXBYTES          0x3B
#define CCxxx0_RCCTRL1_STATUS   0x3C
#define CCxxx0_RCCTRL0_STATUS   0x3D

#define CCxxx0_PATABLE          0x3E
#define CCxxx0_TXFIFO           0x3F
#define CCxxx0_RXFIFO           0x3F





/*******************************************************************************************************
 *******************************************************************************************************
 **************************               Register Bits (c8051f320)           **************************
 *******************************************************************************************************
 *******************************************************************************************************/




//------------------------------------------------------------------------------------------------------
// SMBus

// SMB0CF - SMBus Clock/Configuration Register
#define BM_ENSMB    0x80    // SMBus Enable
#define BM_INH      0x40    // SMBus Slave Inhibit. When this bit is set to logic 1, the SMBus does not
                            // generate an interrupt when slave events occur. This effectively removes 
                            // the SMBus slave from the bus. Master Mode interrupts are not affected
#define BM_BUSY     0x20    // SMBus Busy Indicator. Set to logic 1 by HW when a transfer is in progress.
                            // Cleared when a STOP or free-timeout is sensed.
#define BM_EXTHOLD  0x10    // SMBus Setup and Hold Time Extension Enable.
#define BM_SMBTOE   0x08    // SMBus SCL Timeout Detection Enable. This bit enables SCL low timeout
                            // detection. If set to logic 1, the SMBus forces Timer 3 to reload while
                            // SCL is high and allows Timer 3 to count when SCL goes low. Timer 3 should
                            // be programmed to generate interrupts at 25 ms, and the Timer 3 ISR should
                            // reset SMBus comm.
#define BM_SMBFTE   0x04    // SMBus Free Timeout Detection Enable. When set to 1, the bus will be 
                            // considered free if SCL and SDA remain high for more than 10 SMBus clock
                            // source periods.
#define BM_SMBCS1   0x02    // SMBus Clock Source Selection.
#define BM_SMBCS0   0x01    //      00: Timer 0 Overflow
                            //      01: Timer 1 Overflow
                            //      10: Timer 2 High Byte Overflow
                            //      11: Timer 2 Low Byte Overflow

#define TIMER_0_OVERFLOW            0
#define TIMER_1_OVERFLOW            1
#define TIMER_2_HIGH_BYTE_OVERFLOW  2
#define TIMER_2_LOW_BYTE_OVERFLOW   3

// SMB0CN - SMBus Control Register
#define BM_MASTER   0x80    // SMBus Master/Slave Indicator
                            //      0: SMBus operating in Slave Mode
                            //      1: SMBus operating in Master Mode.
#define BM_TXMODE   0x40    // SMBus Transmit Mode Indicator. This read-only bit indicates when the 
                            // SMBus is operating as a transmitter.
                            //      0: SMBus in Receiver Mode
                            //      1: SMBus in Transmitter Mode.
#define BM_STA      0x20    //  SMBus Start Flag.
                            //      Write:
                            //          0: No Start generated
                            //          1: When operating as a master, a START condition is transmitted 
                            //             if the bus is free (If the bus is not free, the START is
                            //             transmitted after a STOP is received or a timeout is detected).
                            //             If STA is set by SW  as an active Master, a repeated START
                            //             will be generated after the next ACK cycle
                            //      Read:
                            //          0: No Start or repeated Start detected
                            //          1: Start or repeated Start detected.
#define BM_STO      0x10    // SMBus Stop Flag
                            //      Write:
                            //          0: No STOP condition is transmitted
                            //          1: Setting STO to logic 1 causes a STOP condition to be 
                            //             transmitted after the next ACK cycle. When the STOP condition 
                            //             is generated, HW clears STO. If both STA and STO are set, a
                            //             STOP condition is transmitted followed by a START condition
                            //      Read:
                            //          0: No Stop condition detected
                            //          1: Stop condition detected (Slave Mode) or pending (Master Mode)
#define BM_ACKRQ    0x08    // SMBus Acknowledge Request. This read-only bit is set when the SMBus has
                            // received a byte and needs the ACK bit to be written with the correct ACK
                            // response value.
#define BM_ARBLOST  0x04    // SMBus Arbitration Lost Indicator. This read-only bit is set when the
                            // SMBus loses arbitration while operating as a transmitter. A lost
                            // arbitration while a slave indicates a bus error condition.
#define BM_ACK      0x02    // SMBus Acknowledge Flag. This bit defines the out-going ACK level and
                            // records incoming ACK levels. It should be written each time a byte is
                            // received (when ACKRQ=1), or read after each byte is transmitted
                            //      0: A "not acknowledge" has been received (if in Transmitter Mode)
                            //         OR will be transmitted (if in Receiver Mode)
                            //      1: An "acknowledge" has been received (if in Transmitter Mode) OR
                            //         will be transmitted (if in Receiver Mode).
#define BM_SI       0x01    // SMBus Interrupt Flag. Must be cleared by SW. While SI is set, SCL is
                            // held low and the SMBus is stalled.
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// Crossbar

// XBR0 - Port I/O Crossbar Register 0
#define BM_CP1AE        0x80    // Comparator1 Asynchronous Output Enable
#define BM_CP1E         0x40    // Comparator1 Output Enable
#define BM_CP0AE        0x20    // Comparator0 Asynchronous Output Enable
#define BM_CP0E         0x10    // Comparator0 Output Enable
#define BM_SYSCKE       0x08    // SYSCLK Output Enable
#define BM_SMB0E        0x04    // SMBus I/O Enable
#define BM_SPI0E        0x02    // SPI I/O Enable
#define BM_URT0E        0x01    // UART I/O Output Enable

// XBR1 - Port I/O Crossbar Register 1
#define BM_WEAKPUD      0x80    // Port I/O Weak Pull-up Disable:
                                //      0: Weak Pull-ups enabled (except for Ports whose I/O are
                                //         configured as analog input or push-pull output).
                                //      1: Weak Pull-ups disabled.
#define BM_XBAR         0x40    // Crossbar Enable
#define BM_T1E          0x20    // T1 Enable
#define BM_T0E          0x10    // T0 Enable
#define BM_ECIE         0x08    // PCA0 External Counter Input Enable
#define BM_PCA0ME_0     0x00    //      000: All PCA I/O unavailable at Port pins.
#define BM_PCA0ME_1     0x01    //      001: CEX0 routed to Port pin.
#define BM_PCA0ME_2     0x02    //      010: CEX0, CEX1 routed to Port pins.
#define BM_PCA0ME_3     0x03    //      011: CEX0, CEX1, CEX2 routed to Port pins.
#define BM_PCA0ME_4     0x04    //      100: CEX0, CEX1, CEX2, CEX3 routed to Port pins.
#define BM_PCA0ME_5     0x05    //      101: CEX0, CEX1, CEX2, CEX3, CEX4 routed to Port pins.
#define BM_PCA0ME_6     0x06    //      110: Reserved.
#define BM_PCA0ME_7     0x07    //      111: Reserved.
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// SPI Special Function Registers

// SPI0CFG - SPI0 Configuration Register
#define BM_SPIBSY   0x80    // SPI Busy: Set to 1 when a SPI transfer is in progress
#define BM_MSTEN    0x40    // Master Mode Enable: Active High
#define BM_CKPHA    0x20    // SPI0 Clock Phase:
                            //      0: Data centered on first edge of SCK period
                            //      1: Data centered on second edge of SCK period.
#define BM_CKPOL    0x10    // SPI0 Clock Polarity:
                            //      0: SCK low in Idle State
                            //      1: SCK high in Idle State
#define BM_SLVSEL   0x08    // Slave Selected: Set to 1 whenever the NSS pin is low indicating SPI0 is
#define BM_NSSIN    0x04    // the NSS Instantaneous Pin Input
                            // selected slave
#define BM_SRMT     0x02    // Shift Register Empty: Valid in Slave Mode
#define BM_RXBMT    0x01    // Receive Buffer Empty: Valid in Slave Mode

// SPI0CN - SPI0 Control Register
#define BM_SPIF     0x80    // SPI0 Interrupt Flag: Set to 1 at the end of a data transfer. Cleard by SW
#define BM_WCOL     0x40    // Write Collision Flag: Set to 1 to indicate a write to the SPI0 data register 
                            // was attempted while a data transfer was in progress. Cleared by software.
#define BM_MODF     0x20    // Mode Fault Flag: Set when a master mode collision is detected. Cleared by SW
#define BM_RXOVRN   0x10    // Receive Overrun Flag: Set to 1 when the receive buffer still holds unread
                            // data from a previous transfer and the last bit of the current transfer is
                            // shifted into the SPI0 shift register. Cleared by SW.
#define BM_NSSMD1   0x08    // Slave Select Mode
#define BM_NSSMD0   0x04    // Slave Select Mode
                            //      00: 3-Wire Slave or 3-wire Master Mode. NSS signal is not routed to a
                            //          port pin.
                            //      01: 4-Wire Slave or Multi-Master Mode. NSS is always an input to the
                            //          device.
                            //      1x: 4-Wire Single-Master Mode. NSS signal is mapped as an output from
                            //          the device and will assume  the value of NSSMD0
#define BM_TXBMT    0x02    // Transmit Buffer Empty: Set to 0 when new data has been written to the
                            // transmit buffer. When data in the transmit buffer is transferred to the SPI
                            // shift register, this bit will be set to 1, indicating that it is safe to
                            // transmit buffer.
#define BM_SPIEN    0x01    // SPI0 Enable: Active high
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// OSCICN - Internal Oscillator Control Register
#define BM_IOSCEN       0x80    // Internal Oscillator Enable Bit: Active High
#define BM_IFRDY        0x40    // Internal Oscillator Frequency Ready Flag:
                                //      0: Internal Oscillator is not running at programmed frequency.
                                //      1: Internal Oscillator is running at programmed frequency.
#define BM_SUSPEND      0x20    // Force Suspend: SuspendWriting a ‘1’ to this bit will force the internal
                                // oscillator to be stopped. The oscillator will be re-started on the next
                                // non-idle USB event or VBUS interrupt event
#define BM_IFCN1        0x02    // Internal Oscillator Frequency Control Bits
#define BM_IFCN0        0x01    // Internal Oscillator Frequency Control Bits
                                //      00: SYSCLK derived from Internal Oscillator divided by 8
                                //      01: SYSCLK derived from Internal Oscillator divided by 4
                                //      10: SYSCLK derived from Internal Oscillator divided by 2
                                //      11: SYSCLK derived from Internal Oscillator divided by 1.
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// UART0

// SCON0 - Serial Port 0 Control Register
#define BM_S0MODE   0x80    // Serial Port 0 Operation Mode: This bit selects the UART0 Operation Mode.
                            //      0: 8-bit UART with Variable Baud Rate.
                            //      1: 9-bit UART with Variable Baud Rate.
#define BM_MCE0     0x20    // Multiprocessor Communication Enable. The function of this bit is dependent  
                            // on the Serial Port 0 Operation Mode.
                            //  S0MODE = 0: Checks for valid stop bit.
                            //              0: Logic level of stop bit is ignored.
                            //              1: RI0 will only be activated if stop bit is logic level 1.
                            //  S0MODE = 1: Multiprocessor Communications Enable.
                            //              0: Logic level of ninth bit is ignored.
                            //              1: RI0 is set and an interrupt is generated only when the ninth
                            //                 bit is logic 1.
#define BM_REN0     0x10    // Receive Enable. This bit enables/disables the UART receiver.
                            //      0: UART0 reception disabled.
                            //      1: UART0 reception enabled.
#define BM_TB80     0x08    // Ninth Transmission Bit. The logic level of this bit will be assigned to the
                            // ninth transmission bit in 9-bit UART Mode. It is not used in 8-bit UART Mode.
                            // Set or cleared by software as required.
#define BM_RB80     0x04    // Ninth Receive Bit. RB80 is assigned the value of the STOP bit in Mode 0;
                            // it is assigned the value of the 9th data bit in Mode 1.
#define BM_TI0      0x02    // Transmit Interrupt Flag. Set by hardware when a byte of data has been
                            // transmitted by UART0 (after the 8th bit in 8-bit UART Mode, or at the
                            // beginning of the STOP bit in 9-bit UART Mode). When the UART0 interrupt is
                            // enabled, setting this bit causes the CPU to vector to the UART0 interrupt
                            // service routine. This bit must be cleared manually by software.
#define BM_RI0      0x01    // Receive Interrupt Flag. Set to ‘1’ by hardware when a byte of data has
                            // been received by UART0 (set at the STOP bit sam-pling time). When the UART0
                            // interrupt is enabled, setting this bit to ‘1’ causes the CPU to vector to
                            // the UART0 interrupt service routine. This bit must be cleared manually by
                            // software.
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// 4X Clock Multiplyer

// CLKMUL - Clock Multiplyer Control register
#define BM_MULEN    0x80    // Clock Multiplier Enable
                            //      0: Clock Multiplier disabled.
                            //      1: Clock Multiplier enabled.
#define BM_MULINIT  0x40    // Clock Multiplier Initialize. This bit should be a ‘0’ when the Clock
                            // Multiplier is enabled. Once enabled, writing a ‘1’ to this bit will
                            // initialize the Clock Multiplier. The MULRDY bit reads ‘1’ when the Clock
                            // Multiplier is stabilized.
#define BM_MULRDY   0x20    // Clock Multiplier Ready. This read-only bit indicates the status of the
                            // Clock Multiplier.
                            //      0: Clock Multiplier not ready.
                            //      1: Clock Multiplier ready (locked).
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// Timer

// TCON - Timer Control Register
#define BM_TF1  0x80    // Timer 1 Overflow Flag. Set by HW when Timer 1 overflows. Can be cleared by
                        // SW but is automatically cleared when the CPU vectors to the Timer 1 interrupt
                        // service routine.
                        //      0: No Timer 1 overflow detected.
                        //      1: Timer 1 has overflowed.
#define BM_TR1  0x40    // Timer 1 Run Control.
                        //      0: Timer 1 disabled.
                        //      1: Timer 1 enabled.
#define BM_TF0  0x20    // Timer 0 Overflow Flag.
#define BM_TR0  0x10    // Timer 0 Run Control.
#define BM_IE1  0x08    // External Interrupt 1. This flag is set by hardware when an edge/level of type
                        // defined by IT1 is detected. Can be cleared by SW but is automatically cleared 
                        // when the CPU vectors to the External Interrupt 1 service routine if IT1 = 1.
                        // When IT1 = 0, this flag is set to ‘1’ when /INT1 is active as defined by bit
                        // IN1PL in register INT01CF
#define BM_IT1  0x04    // Interrupt 1 Type Select. This bit selects whether the configured /INT1 
                        // interrupt will be edge or level sensitive. /INT1 is con-figured active low or
                        // high by the IN1PL bit in the IT01CF register.
                        //      0: /INT1 is level triggered.
                        //      1: /INT1 is edge triggered.
#define BM_IE0  0x02    // External Interrupt 0. This flag is set by hardware when an edge/level of type
                        // defined by IT0 is detected. Can be cleared by SW but is automatically cleared 
                        // when the CPU vectors to the External Interrupt 0 service routine if IT0 = 1.    
                        // When IT0 = 0, this flag is set to ‘1’ when /INT0 is active as defined by bit
                        // IN0PL in register INT01CF.
#define BM_IT0  0x01    // Interrupt 0 Type Select. This bit selects whether the configured /INT0
                        // interrupt will be edge or level sensitive. /INT0 is con-figured active low or
                        // high by the IN0PL bit in register IT01CF.
                        //      0: /INT0 is level triggered.
                        //      1: /INT0 is edge triggered.

// TMOD - Timer Mode Register
#define BM_GATE1    0x80    // Timer 1 Gate Control.
                            //      0: Timer 1 enabled when TR1 = 1 irrespective of /INT1 logic level.
                            //      1: Timer 1 enabled only when TR1 = 1 AND /INT1 is active as defined 
                            //         by bit IN1PL in register INT01CF
#define BM_C_T1     0x40    // Counter/Timer 1 Select.
                            //      0: Timer Function: Timer 1 incremented by clock defined by T1M bit
                            //      1: Counter Function: Timer 1 incremented by high-to-low transitions on
                            //         external input pin (T1).
#define BM_T1M1     0x20    // Timer 1 Mode Select. These bits select the Timer 1 operation mode.
#define BM_T1M0     0x10    // These bits select the Timer 1 operation mode.
                            //      00: Mode 0; 13-bit counter/timer
                            //      01: Mode 1; 16-bit counter/timer
                            //      10: Mode 2; 8-bit counter/timer with auto-reload
                            //      11: Mode 3; Timer 1 inactive
#define BM_GATE0    0x08    // Timer 0 Gate Control.
                            //      0: Timer 0 enabled when TR0 = 1 irrespective of /INT0 logic level.
                            //      1: Timer 0 enabled only when TR0 = 1 AND /INT0 is active as defined by
                            //         bit IN0PL in register INT01CF.
#define BM_C_T0     0x04    // Counter/Timer Select.
                            //      0: Timer Function: Timer 0 incremented by clock defined by T0M bit
                            //      1: Counter Function: Timer 0 incremented by high-to-low transitions on
                            //         external input pin (T0).
#define BM_T0M1     0x02    // Timer 0 Mode Select.
#define BM_T0M0     0x01    // These bits select the Timer 0 operation mode.
                            // Same as for Timer 1 exept Mode 3: Two 8-bit counter/timers

// TMR3CN: Timer 3 Control Register
#define BM_TF3H     0x80    // Timer 3 High Byte Overflow Flag. Set by hardware when the Timer 3 high byte
                            // overflows from 0xFF to 0x00. In 16 bit mode, this will occur when Timer 3
                            // overflows from 0xFFFF to 0x0000. When the Timer 3 interrupt is enabled,
                            // setting this bit causes the CPU to vector to the Timer 3 ISR. TF3H is not
                            // automatically cleared by HW and must be cleared by SW.
#define BM_TF3L     0x40    // Timer 3 Low Byte Overflow Flag. Set by hardware when the Timer 3 low byte
                            // overflows from 0xFF to 0x00. When this bit is set, an interrupt will be
                            // generated if TF3LEN is set and Timer 3 interrupts are enabled. TF3L will set
                            // when the low byte overflows regardless of the Timer 3 mode. This bit is not
                            // automatically cleared by hard-ware
#define BM_TF3LEN   0x20    // Timer 3 Low Byte Interrupt Enable (Active high). This bit enables/disables
                            // Timer 3 Low Byte interrupts. If TF3LEN is set and Timer 3 interrupts are
                            // enabled, an interrupt will be generated when the low byte of Timer 3
                            // overflows. This bit should be cleared when operating Timer 3 in 16-bit mode
#define BM_T3SOF    0x10    // Timer 3 Start-Of-Frame Capture Enable
                            //      0: SOF Capture disabled
                            //      1: SOF Capture enabled
                            // Each time a USB SOF is received, the contents of the Timer 3 registers
                            // (TMR3H and TMR3L) are latched into the Timer3 reload registers
                            // (TMR3RLH and TMR3RLH), and a Timer 3 interrupt is generated (if enabled).
#define BM_T3SPLIT  0x08    // Timer 3 Split Mode Enable. When this bit is set, Timer 3 operates as
                            // two 8-bit timers with auto-reload
                            //      0: Timer 3 operates in 16-bit auto-reload mode
                            //      1: Timer 3 operates as two 8-bit auto-reload timers
#define BM_TR3      0x04    // Timer 3 Run Control. This bit enables/disables Timer 3. In 8-bit mode,
                            // this bit enables/disables TMR3H only; TMR3L is always enabled in this
                            // mode
                            //      0: Timer 3 disabled
                            //      1: Timer 3 enabled
#define BM_T3XCLK   0x01    // Timer 3 External Clock Select. This bit selects the external clock
                            // source for Timer 3. If Timer 3 is in 8-bit mode, this bit selects the
                            // external oscillator clock source for both timer bytes. However, the
                            // Timer 3 Clock Select bits (T3MH and T3ML in register CKCON) may still be
                            // used to select between the external clock and the system clock for either
                            // timer
                            //      0: Timer 3 external clock selection is the system clock divided by 12
                            //      1: Timer 3 external clock selection is the external clock divided by 8
                            // Note that the external oscillator source divided by 8 is synchronized
                            // with the system clock.

// TMR2CN: Timer 2 Control Register
#define BM_TF2H     0x80    // Timer 2 High Byte Overflow Flag. Set by hardware when the Timer 2 high byte
                            // overflows from 0xFF to 0x00. In 16 bit mode, this will occur when Timer 2
                            // overflows from 0xFFFF to 0x0000. When the Timer 2 interrupt is enabled,
                            // setting this bit causes the CPU to vector to the Timer 2 ISR. TF2H is not
                            // automatically cleared by HW and must be cleared by SW.
#define BM_TF2L     0x40    // Timer 2 Low Byte Overflow Flag. Set by hardware when the Timer 2 low byte
                            // overflows from 0xFF to 0x00. When this bit is set, an interrupt will be
                            // generated if TF2LEN is set and Timer 2 interrupts are enabled. TF2L will set
                            // when the low byte overflows regardless of the Timer 2 mode. This bit is not
                            // automatically cleared by hard-ware
#define BM_TF2LEN   0x20    // Timer 2 Low Byte Interrupt Enable (Active high). This bit enables/disables
                            // Timer 2 Low Byte interrupts. If TF2LEN is set and Timer 2 interrupts are
                            // enabled, an interrupt will be generated when the low byte of Timer 2
                            // overflows. This bit should be cleared when operating Timer 2 in 16-bit mode
#define BM_T2SOF    0x10    // Timer 2 Start-Of-Frame Capture Enable
                            //      0: SOF Capture disabled
                            //      1: SOF Capture enabled
                            // Each time a USB SOF is received, the contents of the Timer 2 registers
                            // (TMR2H and TMR2L) are latched into the Timer2 reload registers
                            // (TMR2RLH and TMR2RLH), and a Timer 3 interrupt is generated (if enabled).
#define BM_T2SPLIT  0x08    // Timer 2 Split Mode Enable. When this bit is set, Timer 2 operates as
                            // two 8-bit timers with auto-reload
                            //      0: Timer 2 operates in 16-bit auto-reload mode
                            //      1: Timer 2 operates as two 8-bit auto-reload timers
#define BM_TR2      0x04    // Timer 2 Run Control. This bit enables/disables Timer 2. In 8-bit mode,
                            // this bit enables/disables TMR2H only; TMR2L is always enabled in this
                            // mode
                            //      0: Timer 2 disabled
                            //      1: Timer 2 enabled
#define BM_T2XCLK   0x01    // Timer 2 External Clock Select. This bit selects the external clock
                            // source for Timer 3. If Timer 2 is in 8-bit mode, this bit selects the
                            // external oscillator clock source for both timer bytes. However, the
                            // Timer 2 Clock Select bits (T2MH and T2ML in register CKCON) may still be
                            // used to select between the external clock and the system clock for either
                            // timer
                            //      0: Timer 2 external clock selection is the system clock divided by 12
                            //      1: Timer 2 external clock selection is the external clock divided by 8
                            // Note that the external oscillator source divided by 8 is synchronized
                            // with the system clock.
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
//  CKCON- Clock Control Register
#define BM_T3MH 0x80    // Timer 3 High Byte Clock Select. This bit selects the clock supplied to the
                        // Timer 3 high byte if Timer 3 is configured in split 8-bit timer mode.
                        // T3MH is ignored if Timer 3 is in any other mode.
                        //      0: Timer 3 high byte uses the clock defined by the T3XCLK bit in TMR3CN.
                        //      1: Timer 3 high byte uses the system clock.
#define BM_T3ML 0x40    // Timer 3 Low Byte Clock Select.This bit selects the clock supplied to Timer 3.
                        // If Timer 3 is configured in split 8-bit timer mode, this bit selects the clock
                        // supplied to the lower 8-bit timer.
                        //      0: Timer 3 low byte uses the clock defined by the T3XCLK bit in TMR3CN.
                        //      1: Timer 3 low byte uses the system clock.
#define BM_T2MH 0x20    // Timer 2 High Byte Clock Select.This bit selects the clock supplied to the
                        // Timer 2 high byte if Timer 2 is configured in split 8-bit timer mode.
                        // T2MH is ignored if Timer 2 is in any other mode.
                        //      0: Timer 2 high byte uses the clock defined by the T2XCLK bit in TMR2CN.
                        //      1: Timer 2 high byte uses the system clock.
#define BM_T2ML 0x10    // Timer 2 Low Byte Clock Select.This bit selects the clock supplied to Timer 2.
                        // If Timer 2 is configured in split 8-bit timer mode, this bit selects the
                        // clock supplied to the lower 8-bit timer.
                        //      0: Timer 2 low byte uses the clock defined by the T2XCLK bit in TMR2CN.
                        //      1: Timer 2 low byte uses the system clock.
#define BM_T1M  0x08    // Timer 1 Clock Select. This select the clock source supplied to Timer 1.
                        // T1M is ignored when C/T1 is set to logic 1.
                        //      0: Timer 1 uses the clock defined by the prescale bits, SCA1-SCA0.
                        //      1: Timer 1 uses the system clock.
#define BM_T0M  0x04    // Timer 0 Clock Select. This bit selects the clock source supplied to Timer 0.
                        // T0M is ignored when C/T0 is set to logic 1.
                        //      0: Counter/Timer 0 uses the clock defined by the prescale bits, SCA1-SCA0.
                        //      1: Counter/Timer 0 uses the system clock.
#define BM_SCA1 0x02    // Timer 0/1 Prescale Bits.
#define BM_SCA0 0x01    // These bits control the division of the clock supplied to Timer 0 and/or
                        // Timer 1 if configured to use prescaled clock inputs.
                        //      00: SYSCLK / 12
                        //      01: SYSCLK / 4
                        //      10: SYSCLK / 48
                        //      11: External clock / 8
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// ADC

// ADC0CF - ADC0 Configuration Register
#define BM_AD0SC4       0x80    // AD0SC4-0: ADC0 SAR Conversion Clock Period Bits. SAR Conversion clock is
#define BM_AD0SC3       0x40    // derived from system clock by the following equation, where AD0SC refers
#define BM_AD0SC2       0x20    // to the 5-bit value held in bits AD0SC4-0. SAR Conversion clock
#define BM_AD0SC1       0x10    // requirements are given in Table 5.1.
#define BM_AD0SC0       0x08    // AD0SC = ((SYSCLK / CLKsar) - 1)
#define BM_AD0LJST      0x04    // ADC0 Left Justify Select
                                //      0:  Data in ADC0H:ADC0L registers are right-justified
                                //      1:  Data in ADC0H:ADC0L registers are left-justified.

// ADC0CN - ADC0 Control Register
#define BM_AD0EN    0x80    // ADC0 Enable Bit
                            //      0:  ADC0 Disabled. ADC0 is in low-power shutdown
                            //      1:  ADC0 Enabled. ADC0 is active and ready for data conversions.
#define BM_AD0TM    0x40    // ADC0 Track Mode Bit
                            //      0:  Normal Track Mode: When ADC0 is enabled, tracking is continuous
                            //          unless a conversion is in progress
                            //      1:  Low-power Track Mode: Tracking Defined by AD0CM2-0 bits
#define BM_AD0INT   0x20    // ADC0 Conversion Complete Interrupt Flag
                            //      0:  ADC0 has not completed a data conversion since the last time
                            //          AD0INT was cleared
                            //      1:  ADC0 has completed a data conversion.
#define BM_AD0BUSY  0x10    // ADC0 Busy Bit
                            //      Read:
                            //          0:  ADC0 conversion is complete or a conversion is not currently
                            //              in progress. AD0INT is set to logic 1 on the falling edge of
                            //              AD0BUSY
                            //          1:  ADC0 conversion is in progress
                            //      Write:
                            //          0:  No Effect
                            //          1:  Initiates ADC0 Conversion if AD0CM2-0 = 000b
#define BM_AD0WINT  0x08    // ADC0 Window Compare Interrupt Flag
                            //      0:  ADC0 Window Comparison Data match has not occurred since this
                            //          flag was last cleared
                            //      1:  ADC0 Window Comparison Data match has occurred.
#define BM_AD0CM2   0x04    // ADC0 Start of Conversion Mode Select.
#define BM_AD0CM1   0x02    // When AD0TM = 0:
#define BM_AD0CM0   0x01    //      000:    ADC0 conversion initiated on every write of ‘1’ to AD0BUSY
                            //      001:    ADC0 conversion initiated on overflow of Timer 0
                            //      010:    ADC0 conversion initiated on overflow of Timer 2
                            //      011:    ADC0 conversion initiated on overflow of Timer 1
                            //      100:    ADC0 conversion initiated on rising edge of external CNVSTR
                            //      101:    ADC0 conversion initiated on overflow of Timer 3
                            //      11x:    Reserved
                            // When AD0TM = 1:
                            //      000:    Tracking initiated on write of ‘1’ to AD0BUSY and lasts 3 SAR
                            //              clocks, followed by conversion
                            //      001:    Tracking initiated on overflow of Timer 0 and lasts 3 SAR
                            //              clocks, followed by conversion
                            //      010:    Tracking initiated on overflow of Timer 2 and lasts 3 SAR
                            //              clocks, followed by conversion
                            //      011:    Tracking initiated on overflow of Timer 1 and lasts 3 SAR
                            //              clocks, followed by conversion
                            //      100:    ADC0 tracks only when CNVSTR input is logic low; conversion
                            //              starts on rising CNVSTR edge
                            //      101:    Tracking initiated on overflow of Timer 3 and lasts 3 SAR
                            //              clocks, followed by conversion
                            //      11x:    Reserved
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// VOLTAGE REFERENCE

// REF0CF - Reference Control Register
#define BM_REFSL    0x08    // Voltage Reference Select.This bit selects the source for the internal
                            // voltage reference
                            //      0:  VREF pin used as voltage reference
                            //      1:  VDD used as voltage reference
#define BM_TEME     0x04    // Temperature Sensor Enable Bit
                            //      0:  Internal Temperature Sensor off
                            //      1:  Internal Temperature Sensor on
#define BM_BIASE    0x02    // Internal Analog Bias Generator Enable Bit
                            //      0:  Internal Bias Generator off
                            //      1:  Internal Bias Generator on
#define BM_REFBE    0x01    // Internal Reference Buffer Enable Bit
                            //      0:  Internal Reference Buffer disabled
                            //      1:  Internal Reference Buffer enabled. Internal voltage reference
                            //          driven on the VREF pin
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// INTERRUPT

// EIP1 - Extended Interrupt Priority 1
#define BM_PT3      0x80    // Timer 3 Interrupt Priority Control
#define BM_PCP1     0x40    // Comparator1 (CP1) Interrupt Priority Control
#define BM_PCP0     0x20    // Comparator0 (CP0) Interrupt Priority Control
#define BM_PPCA0    0x10    // Programmable Counter Array (PCA0) Interrupt Priority Control
#define BM_PADC0    0x08    // ADC0 Conversion Complete Interrupt Priority Control
#define BM_PWADC0   0x04    // ADC0 Window Comparator Interrupt Priority Control
#define BM_PUSB0    0x02    // USB0 Interrupt Priority Control
#define BM_PSMB0    0x01    // SMBus (SMB0) Interrupt Priority Control

// EIP2 - Extended Interrupt Priority 2
#define BM_PVBUS    0x01    // VBUS Level Interrupt Priority Control

// EIE1: Extended Interrupt Enable
#define BM_ET3      0x80    // Enable Timer 3 Interrupt
#define BM_ECP1     0x40    // Enable Comparator1 (CP1) Interrupt
#define BM_ECP0     0x20    // Enable Comparator0 (CP0) Interrupt
#define BM_EPCA0    0x10    // Enable Programmable Counter Array (PCA0) Interrupt
#define BM_EADC0    0x08    // Enable ADC0 Conversion Complete Interrupt
#define BM_EWADC0   0x04    // Enable Window Comparison ADC0 Interrupt
#define BM_EUSB0    0x02    // Enable USB0 Interrupt
#define BM_ESMB0    0x01    // Enable SMBus (SMB0) Interrupt

// EIE2 - Extended Interrupt Priority 2
#define BM_EVBUS    0x01    // Enable VBUS Level Interrupt
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// COMPARATOR

// CPT0CN: Comparator0 Control Register
#define BM_CP0EN        0x80    // Comparator0 Enable Bit (Active high)
#define BM_CP0OUT       0x40    // Comparator0 Output State Flag
                                //      0: Voltage on CP0+ < CP0-
                                //      1: Voltage on CP0+ > CP0-
#define BM_CP0RIF       0x20    // Comparator0 Rising-Edge Flag
                                //      0: No Comparator0 Rising Edge has occurred since this flag was last
                                //         cleared
                                //      1: Comparator0 Rising Edge has occurred
#define BM_CP0FIF       0x10    // Comparator0 Falling-Edge Flag
                                //      0: No Comparator0 Falling-Edge has occurred since this flag was last
                                //         cleared
                                //      1: Comparator0 Falling-Edge Interrupt has occurred
#define BM_CP0HYP1      0x08    // Comparator0 Positive Hysteresis Control Bits
#define BM_CP0HYP0      0x04    //      00: Positive Hysteresis Disabled
                                //      01: Positive Hysteresis = 5 mV
                                //      10: Positive Hysteresis = 10 mV
                                //      11: Positive Hysteresis = 20 mV
#define BM_CP0HYN1      0x02    // Comparator0 Negative Hysteresis Control Bits
#define BM_CP0HYN0      0x01    //      00: Negative Hysteresis Disabled
                                //      01: Negative Hysteresis = 5 mV
                                //      10: Negative Hysteresis = 10 mV
                                //      11: Negative Hysteresis = 20 mV

// CPT1CN: Comparator1 Control Register
#define BM_CP1EN        0x80    // Comparator1 Enable Bit (Active high)
#define BM_CP1OUT       0x40    // Comparator1 Output State Flag
                                //      0: Voltage on CP0+ < CP0-
                                //      1: Voltage on CP0+ > CP0-
#define BM_CP1RIF       0x20    // Comparator1 Rising-Edge Flag
                                //      0: No Comparator0 Rising Edge has occurred since this flag was last
                                //         cleared
                                //      1: Comparator0 Rising Edge has occurred
#define BM_CP1FIF       0x10    // Comparator1 Falling-Edge Flag
                                //      0: No Comparator0 Falling-Edge has occurred since this flag was last
                                //         cleared
                                //      1: Comparator0 Falling-Edge Interrupt has occurred
#define BM_CP1HYP1      0x08    // Comparator1 Positive Hysteresis Control Bits
#define BM_CP1HYP0      0x04    //      00: Positive Hysteresis Disabled
                                //      01: Positive Hysteresis = 5 mV
                                //      10: Positive Hysteresis = 10 mV
                                //      11: Positive Hysteresis = 20 mV
#define BM_CP1HYN1      0x02    // Comparator1 Negative Hysteresis Control Bits
#define BM_CP1HYN0      0x01    //      00: Negative Hysteresis Disabled
                                //      01: Negative Hysteresis = 5 mV
                                //      10: Negative Hysteresis = 10 mV
                                //      11: Negative Hysteresis = 20 mV
//------------------------------------------------------------------------------------------------------


//------------------------------------------------------------------------------------------------------
// PROGRAMMABLE COUNTER ARRAY (PCA0)

// PCA0MD: PCA Mode Register
#define BM_CIDL     0x80    // CIDL: PCA Counter/Timer Idle Control. Specifies PCA behavior when CPU is
                            // in Idle Mode
                            //      0: PCA continues to function normally while the system controller
                            //         is in Idle Mode
                            //      1: PCA operation is suspended while the system controller is in
                            //         Idle Mode
#define BM_WDTE     0x40    // WDTE: Watchdog Timer Enable. If this bit is set, PCA Module 4 is used as
                            // the watchdog timer
                            //      0: Watchdog Timer disabled
                            //      1: PCA Module 4 enabled as Watchdog Timer
#define BM_WDLCK    0x20    // WDLCK: Watchdog Timer LockThis bit enables and locks the Watchdog Timer.
                            // When WDLCK is set to ‘1’, the Watchdog Timer may not be disabled until
                            // the next system reset
                            //      0: Watchdog Timer unlocked
                            //      1: Watchdog Timer enabled and locked
#define BM_CPS2     0x08    // Counter/Timer Pulse Select. These bits select the timebase source for
#define BM_CPS1     0x04    // the PCA counter
#define BM_CPS0     0x02    //      000: System clock divided by 12
                            //      001: System clock divided by 4
                            //      010: Timer 0 overflow
                            //      011: High-to-low transitions on ECI (max rate = SYS CLK / 4
                            //      100: System clock
                            //      101: External clock divided by 8
                            //      110: Reserved
                            //      111: Reserved
#define BM_ECF      0x01    // ECF: PCA Counter/Timer Overflow Interrupt Enable. This bit sets the
                            // masking of the PCA Counter/Timer Overflow (CF) interrupt
                            //      0: Disable the CF interrupt
                            //      1: Enable a PCA Counter/Timer Overflow interrupt request when CF
                            //         (PCA0CN.7) is set.

// PCA0CPMn: PCA Capture/Compare Mode Registers
#define BM_PWM16    0x80    // 16-bit Pulse Width Modulation Enable. This bit selects 16-bit mode when
                            // Pulse Width Modulation mode is enabled (PWMn = 1)
                            //      0: 8-bit PWM selected
                            //      1: 16-bit PWM selected
#define BM_ECOM     0x40    // ECOMn: Comparator Function Enable. This bit enables/disables the
                            // comparator function for PCA module n
                            //      0: Disabled
                            //      1: Enabled
#define BM_CAPP     0x20    // CAPPn: Capture Positive Function Enable. This bit enables/disables the
                            // positive edge capture for PCA module n
                            //      0: Disabled
                            //      1: Enabled
#define BM_CAPN     0x10    // CAPNn: Capture Negative Function Enable. This bit enables/disables the
                            // negative edge capture for PCA module n
                            //      0: Disabled
                            //      1: Enabled
#define BM_MAT      0x08    // MATn: Match Function Enable. This bit enables/disables the match function
                            // for PCA module n. When enabled, matches of the PCA counter with a
                            // module's capture/compare register cause the CCFn bit in PCA0MD register
                            // to be set to logic 1
                            //      0: Disabled
                            //      1: Enabled
#define BM_TOG      0x04    // TOGn: Toggle Function Enable. This bit enables/disables the toggle
                            // function for PCA module n. When enabled, matches of the PCA counter with
                            // a module's capture/compare register cause the logic level on the CEXn pin
                            // to toggle. If the PWMn bit is also set to logic 1, the module operates
                            // in Frequency Output Mode
                            //      0: Disabled
                            //      1: Enabled
#define BM_PWM      0x02    // PWMn: Pulse Width Modulation Mode Enable. This bit enables/disables
                            // the PWM function for PCA module n. When enabled, a pulse width modulated
                            // signal is output on the CEXn pin. 8-bit PWM is used if PWM16n is
                            // cleared; 16-bit mode is used if PWM16n is set to logic 1. If the TOGn
                            // bit is also set, the module operates in Frequency Output Mode
                            //      0: Disabled
                            //      1: Enabled
#define BM_ECCF     0x01    // ECCFn: Capture/Compare Flag Interrupt Enable. This bit sets the masking
                            // of the Capture/Compare Flag (CCFn) interrupt
                            //      0: Disable CCFn interrupts
                            //      1: Enable a Capture/Compare Flag interrupt request when CCFn is set
//------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************               BIT accessible Registers            **************************
 *******************************************************************************************************
 *******************************************************************************************************/




//------------------------------------------------------------------------------------------------------
// P0
sbit P0_0 = P0^0;
sbit P0_1 = P0^1;
sbit P0_2 = P0^2;
sbit P0_3 = P0^3;
sbit P0_4 = P0^4;
sbit P0_5 = P0^5;
sbit P0_6 = P0^6;
sbit P0_7 = P0^7;

// P1
sbit P1_0 = P1^0;
sbit P1_1 = P1^1;
sbit P1_2 = P1^2;
sbit P1_3 = P1^3;
sbit P1_4 = P1^4;
sbit P1_5 = P1^5;
sbit P1_6 = P1^6;
sbit P1_7 = P1^7;

// P2
sbit P2_0 = P2^0;
sbit P2_1 = P2^1;
sbit P2_2 = P2^2;
sbit P2_3 = P2^3;
sbit P2_4 = P2^4;
sbit P2_5 = P2^5;
sbit P2_6 = P2^6;
sbit P2_7 = P2^7;

// P3
sbit P3_0 = P3^0;
//------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                 ISR vector addresses              **************************
 *******************************************************************************************************
 *******************************************************************************************************/




//------------------------------------------------------------------------------------------------------
// Usage example:
//      void my_serial_isr() interrupt INUM_UART0 {
//          ...
//      }
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// ISR servicing external interrupt 0
#define INUM_INT0           0
#define INUM_EXTERNAL0      0

// ISR servicing timer 0
#define INUM_TIMER0         1

// ISR servicing external interrupt 1
#define INUM_INT1           2
#define INUM_EXTERNAL1      2

// ISR servicing timer 1
#define INUM_TIMER1         3

//ISR servicing serial port 0
#define INUM_SERIAL0        4
#define INUM_UART0          4
#define INUM_UART0_RX       40      // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_UART0_TX       41      // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing timer 2
#define INUM_TIMER2         5
#define INUM_TIMER2_H       50      // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_TIMER2_L       51      // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing SPI
#define INUM_SPI0           6
#define INUM_SPI0_IF        60      // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_SPI0_WCOL      61      // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_SPI0_MODF      62      // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_SPI0_RXOVRN    63      // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing SMB
#define INUM_SMB0           7

// ISR servicing USB
#define INUM_USB0           8

// ISR servicing ADC0 window compare
#define INUM_ADC0WINDOW     9

// ISR servicing ADC0 conversion complete
#define INUM_ADC0COMP       10
#define INUM_ADC0           10

// ISR servicing programmable counter array
#define INUM_PCA0           11
#define INUM_PCA0_IF        110     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_PCA0_CCF0      111     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_PCA0_CCF1      112     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_PCA0_CCF2      113     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_PCA0_CCF3      114     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_PCA0_CCF4      115     // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing comparator 0
#define INUM_ACOMP0         12
#define INUM_ACOMP0_F       120     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_ACOMP0_R       121     // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing comparator 1
#define INUM_ACOMP1         13
#define INUM_ACOMP1_F       130     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_ACOMP1_R       131     // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing timer 3
#define INUM_TIMER3         14
#define INUM_TIMER3_H       140     // Can not be an ISR (Used to identify second-level interrupt))
#define INUM_TIMER3_L       141     // Can not be an ISR (Used to identify second-level interrupt))

// ISR servicing VBUS level
#define INUM_VBUS           15
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// V7.50
sbit ESPI0  = IE^6; // Enable Serial Peripheral Interface (SPI0) Interrupt
sfr EIP1    = 0xF6; // External interrupt enable 1                             
sfr EIE2    = 0xE7; // External interrupt enable 2     
sbit PSPI0  = IP^6; // Serial Peripheral Interface (SPI0) Interrupt Priority Control.
sfr EIP2    = 0xF7; // Extended Interrupt Priority 2          
//------------------------------------------------------------------------------------------------------




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: regssrf04.h,v $
 * Revision 1.6  2007/06/22 13:29:38  a0190596
 * Added
 *  // V7.50
 * sbit ESPI0  = IE^6; // Enable Serial Peripheral Interface (SPI0) Interrupt
 * sfr EIP1    = 0xF6; // External interrupt enable 1
 * sfr EIE2    = 0xE7; // External interrupt enable 2
 * sbit PSPI0  = IP^6; // Serial Peripheral Interface (SPI0) Interrupt Priority Control.
 * sfr EIP2    = 0xF7; // Extended Interrupt Priority 2
 *
 * since revision 2.1 of the C8051F320.h file is not available from the web (it comes with uVision3)
 *
 * Revision 1.5  2006/11/17 10:37:55  a0190596
 * Added register 0x3C and 0x3D
 *
 * Revision 1.4  2006/10/05 13:50:34  a0190596
 * The following has been removed to be compliant with revision 2.1 of the C8051F320.h file:
 *
 * //------------------------------------------------------------------------------------------------------
 * // V7.50
 * //sbit ESPI0  = IE^6; // Enable Serial Peripheral Interface (SPI0) Interrupt
 * //sfr EIP1    = 0xF6; // External interrupt enable 1
 * //sfr EIE2    = 0xE7; // External interrupt enable 2
 * //sbit PSPI0  = IP^6; // Serial Peripheral Interface (SPI0) Interrupt Priority Control.
 * //sfr EIP2    = 0xF7; // Extended Interrupt Priority 2
 * //------------------------------------------------------------------------------------------------------
 *
 * Revision 1.3  2006/03/31 13:36:52  a0190596
 * Added the following:
 * bit PSPI0  = IP^6; // Serial Peripheral Interface (SPI0) Interrupt Priority Control.
 * sfr EIP2    = 0xF7; // Extended Interrupt Priority 2
 *
 * Revision 1.2  2005/02/07 14:35:02  sna
 * Initial version in CVS.
 *
 *
 *******************************************************************************************************/




#endif//REGSSRF04_H
