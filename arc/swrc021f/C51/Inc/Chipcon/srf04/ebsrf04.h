/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains macros and function that are useful when                *
 *      ***   +             accessing peripherals on the SmartRF04EB using the c8051f320               *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     ebsrf04.h                                                                  *
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
#ifndef EBSRF04_H
#define EBSRF04_H
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\common.h>
#include <intrins.h>





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                       I/O PORTS                   **************************
 *******************************************************************************************************
 *******************************************************************************************************/
//------------------------------------------------------------------------------------------------------
// Port initialization
// This section contain a routine for configuring the 80512f320 I/O ports to communicate with
// the SmartRF04EB peripherals.
// NOTE: Some peripheral I/O pins can function as either inputs or outputs, depending on the
// configuration of the peripheral. By default, the configuration utility will configure these I/O pins
// as push-pull outputs.
//
// Port I/O initialization consists of the following steps:
//
// Step 1.  Select the input mode (analog or digital) for all Port pins, using the Port Input Mode
//          register (PnMDIN). ‘1’ indicates a digital input, and ‘0’ indicates an analog input.
//          All pins default to digital inputs on reset.
//
// Step 2.  Select the output mode (open-drain or push-pull) for all Port pins, using the Port
//          Output Mode register (PnMDOUT). ‘1’ indicates that output is push-pull, and ‘0’ indicates
//          open-drain. All pins default to open-drain on reset
//
// Step 3.  Select any pins to be skipped by the I/O Crossbar using the Port Skip registers (PnSKIP).
//              0: Corresponding P0.n pin is not skipped by the Crossbar.
//              1: Corresponding P0.n pin is skipped by the Crossbar.
//          All analog input pins should be configured to be skipped by the Crossbar (p 131)
//          Set to '0' on reset
//
// Step 4.  Assign Port pins to desired peripherals (XBR0, XBR1).
//
// Step 5.  Enable the Crossbar (XBARE = ‘1’).
//
// All Port pins must be configured as either analog or digital inputs. Any pins to be used as Comparator
// or ADC inputs should be configured as an analog inputs. When a pin is configured as an analog input,
// its weak pull-up, digital driver, and digital receiver are disabled. This process saves power and
// reduces noise on the analog input. Pins configured as digital inputs may still be used by analog
// peripherals; however this practice is not recommended. To configure a Port pin for digital input,
// write ‘0’ to the corresponding bit in register PnMDOUT, and write ‘1’ to the corresponding Port latch
// (register Pn).
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// Port definition SmartRF04Eb 1.9

// Port 0
#define SCLK_           0x01    // P0.0     SPI Serial clock                            	Output
#define SO_             0x02    // P0.1     SPI MISO signal, xx00 SO/GDO1               	Output
#define GDO1_           0x02    // P0.1
#define SI_             0x04    // P0.2     SPI MOSI signal, xx00 SI                    	Input
#define CSn_            0x08    // P0.3     SPI slave select signal                     	Output
#define UART_TD_        0x10    // P0.4     UART TD                                     	Output
#define UART_RD_        0x20    // P0.5     UART RD                                     	Input
#define GDO0_           0x40    // P0.6     xx00 GDO0                                   	Input
#define GDO2_           0x80    // P0.7     xx00 GDO2                                   	Input

// Port 1
#define SDA_            0x01    // P1.0     SMBus data (for LCD)                        	Output
#define SCL_            0x02    // P1.1     SMBus clock (for LCD)                       	Output
#define MIC_INPUT_      0x04    // P1.2     Audio input                                 	Input (analog)
#define BUTTON_PUSH_    0x08    // P1.3     Button pushed                               	Input
#define PWM_OUTPUT_     0x10    // P1.4     PWM audio output                            	Output
#define CTS_            0x20    // P1.5     Clear To Send (MCU to terminal)             	Output
#define JOY_PUSH_       0x20    // P1.5     Joustick push button                        	Input
#define POT_            0x40    // P1.6     Potmeter input                              	Input (analog)
#define LED1_           0x80    // P1.7     LED1, (Green), active low                   	Output

// Port 2
#define LED2_           0x01    // P2.0     LED2 (Red), active low                      	Output
#define JOY_            0x02    // P2.1     Joystick input (analog coded voltage)       	Input (analog)
#define SoC_PRESENT_    0x04    // P2.2     Tells USB MCU whether a Soc is present or not 	Input
#define RESET_N_        0x08    // P2.3     LCD Power on reset signal, ccxx10 RESET     	Output
#define LED4_           0x10    // P2.4     LED4 (Blue), active low                     	Output
#define CP_ENABLE_   	0x20    // P2.5     Turns on/off the RS-232 on-board power supply   Output
#define RTS_            0x40    // P2.6     Request To send (terminal to MCU)             	Input
#define LED3_           0x80    // P2.7     LED3 (Yellow), active low                   	Output

// Macro to set up the crossbar and I/O ports to communicate with the SmartRF04EB peripherals
#define IO_PORT_INIT() \
    do { \
        XBR0 = (BM_SMB0E | BM_SPI0E | BM_URT0E); \
        XBR1 = (BM_XBAR | BM_PCA0ME_1  | BM_WEAKPUD); \
        P0MDOUT = UART_TD_ | UART_RD_ |  SCLK_ | SO_ | SI_ | CSn_; \
        P1MDOUT = LED1_ | PWM_OUTPUT_; \
        P2MDOUT = LED2_ | LED3_ | LED4_ | RESET_N_; \
        P1MDIN = (~POT_) & (~MIC_INPUT_); \
        P2MDIN = (~JOY_); \
        P0SKIP = GDO0_ | GDO2_;\
        P1SKIP = POT_ | MIC_INPUT_ | BUTTON_PUSH_; \
        P2SKIP = JOY_; \
    } while (0)
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// Port 0
#define P_SCLK          P0_0
#define P_SO            P0_1
#define P_GDO1          P0_1
#define P_SI            P0_2
#define P_CSn           P0_3
#define P_UART_TD       P0_4
#define P_UART_RD       P0_5
#define P_GDO0          P0_6
#define P_GDO2          P0_7

// Port 1
#define P_SDA           P1_0
#define P_SCL           P1_1
#define P_MIC_INPUT     P1_2
#define P_BUTTON_PUSH   P1_3
#define P_PWM_OUTPUT    P1_4
#define P_JOY_PUSH      P1_5
#define P_CTS			P1_5
#define P_POT           P1_6
#define P_LED1          P1_7

// Port 2
#define P_LED2          P2_0
#define P_JOY           P2_1
#define P_SoC_PRESENT   P2_2
#define P_RESET_N       P2_3
#define P_LED4          P2_4
#define P_CP_ENABLE     P2_5
#define P_RTS      		P2_6
#define P_LED3          P2_7
//------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                        LEDS                       **************************
 *******************************************************************************************************
 *******************************************************************************************************/
//------------------------------------------------------------------------------------------------------
// This section contain macros for turning on and off the 4 LEDS on the SmartRF04EB.
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// Useful defines
#define LED_ON  0
#define LED_OFF 1
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// Set LED
#define SET_GLED(x)         P_LED1 = x
#define SET_RLED(x)         P_LED2 = x
#define SET_YLED(x)         P_LED3 = x
#define SET_BLED(x)         P_LED4 = x
//------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************             BUTTONS/JOYSTICK/POTMETER             **************************
 *******************************************************************************************************
 *******************************************************************************************************/
//------------------------------------------------------------------------------------------------------
// This section contains useful defines and macros/functions for monitoring the state of the joystick
// and the push button (S1)
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
#define BUTTON_PUSHED()     (!P_BUTTON_PUSH)
#define JOY_PUSHED()        (P_JOY_PUSH)  
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
//  BOOL ebButtonPushed()
//
//  DESCRIPTION:
//      This function detects if the button is being pushed. The function implements software debounce.
//   
//  RETURN TYPE:
//      BOOL
//          TRUE: Button is being pushed
//          FALSE: Button is not being pushed
//------------------------------------------------------------------------------------------------------
BOOL ebButtonPushed(void);




//------------------------------------------------------------------------------------------------------
//  BOOL ebJoyPushed()
//
//  DESCRIPTION:
//      This function detects if the joystick button is being pushed.
//      The function implements software debounce.
//   
//  RETURN TYPE:
//      BOOL
//          TRUE: Joystick button is being pushed
//          FALSE: Joystick button is not being pushed
//------------------------------------------------------------------------------------------------------
BOOL ebJoyPushed(void);




//------------------------------------------------------------------------------------------------------
// Defines used as input to the ebAdcInit function
#define ADC_AUDIO           0x02
#define ADC_POT             0x06
#define ADC_JOY             0x09
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
//  void ebAdcInit(UINT8 adcInput)
//
//  DESCRIPTION:
//      Function used to initialize ADC before using the ebGetJoystickPosition or ebReadPotentiometer
//      functions.
//
//  ARGUMENTS:
//      UINT8 adcInput
//          ADC_JOY
//          ADC_POT
//          ADC_AUDIO
//------------------------------------------------------------------------------------------------------
void ebAdcInit(UINT8 adcInput);




//------------------------------------------------------------------------------------------------------
// Defines used as return values by the getJoystickAction function described below
#define JOYSTICK_CENTER  0
#define JOYSTICK_UP      1
#define JOYSTICK_DOWN    2
#define JOYSTICK_LEFT    3
#define JOYSTICK_RIGHT   4
#define JOYSTICK_PUSHED  5
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
//  UINT8 ebGetJoystickPosition(void)
//
//  DESCRIPTION:
//      This function will read the ADC to determine the current joystick position. ebAdcInit(ADC_JOY)
//      must be called before this function can be called.
//
//      RETURN VALUE:
//          UINT8
//              JOYSTICK_CENTER  0
//              JOYSTICK_UP      1
//              JOYSTICK_DOWN    2
//              JOYSTICK_LEFT    3
//              JOYSTICK_RIGHT   4            
//              JOYSTICK_PUSHED  5
//------------------------------------------------------------------------------------------------------
UINT8 ebGetJoystickPosition(void);




//------------------------------------------------------------------------------------------------------
//  UINT8 ebReadPotentiometer(void)
//
//  DESCRIPTION:
//      This function reads the potmeter located at the SmartRF04EB using the ADC. The function only
//      reads the 8 MSBs from the ADC. The ebAdcinit(ADC_POT) function must be called before this
//      function can be called.
//
//  RETURN VALUE:
//      UINT8
//          8 bit sample from ADC
//------------------------------------------------------------------------------------------------------
UINT8 ebReadPotentiometer(void);





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                RS-232 Transceivers                **************************
 *******************************************************************************************************
 *******************************************************************************************************/
//-------------------------------------------------------------------------------------------------------
// This section contains macros for turning on/off the RS-232 on-board power supply, 
// enable/disable hardware flow control (set the direction of the CTS/JOY_PUSH pin), and
// for setting/clearing the CTS output
// 		RS_232_FORCE_ON()
//		RS_232_FORCE_OFF()
//		HARDWARE_FLOW_CONTROL_ENABLE()
//		HARDWARE_FLOW_CONTROL_DISABLE()
//		UART_CTS_FLOW_ENABLE()
//		UART_CTS_FLOW_DISABLE()
//-------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
// Macro for turning on/off the RS-232 on-board power supply
#define RS_232_FORCE_ON()		P_CP_ENABLE = 1
#define RS_232_FORCE_OFF()		P_CP_ENABLE = 0

// Enable Hardware Flow Control. CTS is set as an output. It is no longer possible to use the
// joystick push button 
#define HARDWARE_FLOW_CONTROL_ENABLE() \
	do { \
		UINT8 timeout = 30; \
    	P1MDOUT |= CTS_; \
		do { \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
    	} while (--timeout); \
    } while (0)

// Disable Hardware Flow Control. CTS is set as an input.It is now possible to use the 
// joystick push button (it shares the same pin as CTS)
#define HARDWARE_FLOW_CONTROL_DISABLE() \
	do { \
		UINT8 timeout = 30; \
    	P1MDOUT &= ~CTS_; \
		do { \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
        	_nop_(); \
    	} while (--timeout); \
    } while (0) 

#define UART_CTS_FLOW_ENABLE()    (P_CTS = 0)
#define UART_CTS_FLOW_DISABLE()   (P_CTS = 1)
//------------------------------------------------------------------------------------------------------





/*******************************************************************************************************
 *******************************************************************************************************
 **************************                    LCD Display                    **************************
 *******************************************************************************************************
 *******************************************************************************************************/
//------------------------------------------------------------------------------------------------------
// This section contains everything that is needed for writing to the LCD display using interrupt.
// Please see the Lcd.c file for example on how to write to the display.
//      void ebLcdInit(void)
//      BYTE ebLcdUpdate(UINT8 *pLine1, UINT8 *pLine2)
//------------------------------------------------------------------------------------------------------




//------------------------------------------------------------------------------------------------------
//  void ebLcdInit(void)
//
//  DESCRIPTION:
//      Function used to initialize the LCD display. The SMBus uses timer 0 to generate SCL
//      This function also writes 32 SPACE characters to the display to clear it.
//------------------------------------------------------------------------------------------------------
void ebLcdInit(void);




//------------------------------------------------------------------------------------------------------
//  BYTE ebLcdUpdate(UINT8 *pLine1, UINT8 *pLine2)
//
//  DESCRIPTION:
//      This function converts the two text strings from ASCII to the character set used by the LCD
//      display. It also sets up the i2cBuffer corretly with slave address, register select, etc,
//      for accessing both lines of the display.
//      A complete LCD display update is done in four I2C write operations:
//          1. Move to the first character on the first line
//          2. Write 16 characters to the display
//          3. Move to the first character on the second line
//          4. Write another 16 characters to the display
//
//  ARGUMENTS:
//      UINT8 *pLine1
//          Pointer to text string to be written to line 1
//      UINT8 *pLine2
//          Pointer to text string to be written to line 2
//
//  RETURN VALUE:
//      BYTE
//          FALSE:  A transfer is already in progress and nothing is done
//          TRUE:   A new transfer is started  
//------------------------------------------------------------------------------------------------------
BYTE ebLcdUpdate(UINT8 *pLine1, UINT8 *pLine2);




/*******************************************************************************************************
 * Revision history:                                                                                   *
 *
 * $Log: ebsrf04.h,v $
 * Revision 1.3  2005/10/25 09:47:35  sna
 * Added macros for HW flow control (UART)
 * Changed names on some input/output pins
 *
 * Revision 1.2  2005/02/07 14:34:48  sna
 * Initial version in CVS.
 *
 *
 ******************************************************************************************************/




#endif// EBSRF04.H













