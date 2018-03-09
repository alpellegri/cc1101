/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This file contains all functions related to menu handeling in the          *
 *      ***   +             InfiniteLink example.                                                      *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     Menu.c                                                                     *
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
#include <InfiniteLink.h>




//-------------------------------------------------------------------------------------------------------
// Global variables
MENU_DATA xdata menuData = {
    270,    // Packet length
    100,    // Number of packets
    TX      // Radio Mode
};

MENU_ITEM code menuTable[] = {
  //{   menuName,               value,                      min,    max     }
    {   "Set pkt. length",      &menuData.packetLength,     270,    450     },
    {   "Number of packets",    &menuData.numberOfPackets,  100,    1000    },
    {   "Radio Mode",           &menuData.radioMode,        0,      1       },
    {   "Start",                NULL,                       0,      0       }
};

//Variable to keep track of the current menu entry (can have a value between 0 and 5)
UINT8 index = 0; 

const UINT8 menuDepth = sizeof(menuTable) / sizeof(MENU_ITEM);

// Byte array used by the intToAscii(UINT16 value) function
UINT8 xdata asciiString[6];
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void parseMenu(UINT8 joystickPosition) 
//
//  DESCRIPTION:
//      This function allows the user to set different parameters by using the joystick to navigate
//      through a menu. The function is called every time the joystick position is changed. 
//
//          |                     Menu Entries        |   Values (currentMenuEntry.pValue)
//         |||                    --------------------------------------------------------
//        |||||   
//          |     index = 0       Packet Length       |   270 , 290, ...... ...., 430, 450
// Joystick |     index = 1       Number of Packets   |   100, 200, 300, ......, 900, 1000
// up/down  |     index = 2       Radio Mode          |   Rx, Tx
//          |     index = 3       Start               |   NULL
//        |||||   
//         |||                    --------------------------------------------------------
//          |                                               |                          |
//                                                         ||    Joystick left/right   ||
//                                                        |||--------------------------|||
//                                                         ||                          ||
//                                                          |                          |
//-------------------------------------------------------------------------------------------------------
void parseMenu(UINT8 joystickPosition) {
    MENU_ITEM xdata currentMenuEntry;

    currentMenuEntry = menuTable[index];

    switch (joystickPosition) {

        case JOYSTICK_UP:

            // Change Menu Item
            if (index > 0)
                index--;
            break;

        case JOYSTICK_DOWN:

            // Change Menu Item
            if (index < (menuDepth - 1))
                index++;
            break;

        case JOYSTICK_LEFT:

            // Decrease value of the current manu item unless value already has reached its minimum
            if (*(currentMenuEntry.pValue) > currentMenuEntry.min) {
                if (index == PACKET_LENGTH)
                    (*(currentMenuEntry.pValue)) -= 20;
                else if (index == NUMBER_OF_PACKETS)
                    (*(currentMenuEntry.pValue)) -= 100;
                else
                    (*(currentMenuEntry.pValue))--;
            }
            break;

        case JOYSTICK_RIGHT:

            // Increase value of the current manu item unless value already has reached its maximum
            if (*(currentMenuEntry.pValue) < currentMenuEntry.max) {
                if (index == PACKET_LENGTH)
                    (*(currentMenuEntry.pValue)) += 20;
                else if (index == NUMBER_OF_PACKETS)
                    (*(currentMenuEntry.pValue)) += 100;
                else
                    (*(currentMenuEntry.pValue))++;
            }
            break;

        default:
            break;
    }
    handleAndDisplayMenu();
} //parseMenu




//-------------------------------------------------------------------------------------------------------
//  void handleAndDisplayMenu(void) 
//
//  DESCRIPTION:
//      This function is called every time there has been a change in the joystick position while in the
//      SETUP state. The function display the proper text on the LCD display depending on the current
//      menu entry. In two cases (current menu entry = FEC or WHITENING) this function also updates 
//      configuration registers based on menu settings (enable/disable FEC affects the MDMCFG1 register, 
//      enable/disable WHITENING affects the PKTCTRL0 register).
//      The function is also called once before the infinite loop in the main function.
//-------------------------------------------------------------------------------------------------------
void handleAndDisplayMenu(void) {
    MENU_ITEM xdata currentMenuEntry;

    currentMenuEntry = menuTable[index];

    switch (index) {
    
        case RADIO_MODE:
            if (menuData.radioMode == RX)
                ebLcdUpdate(currentMenuEntry.menuName, "Rx");
            else {
                ebLcdUpdate(currentMenuEntry.menuName, "Tx");
            }
            break;

        case START:
            ebLcdUpdate("Press S1", "to start");
            break;

        default:    // PACKET_LENGTH or NUMMBER_OF_PACKETS
            intToAscii((*(currentMenuEntry.pValue)));
            ebLcdUpdate(currentMenuEntry.menuName, asciiString);
            break;
    }
} //handleAndDisplayMenu




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: Menu.c,v $
 * Revision 1.3  2007/11/23 09:24:40  a0190596
 * no message
 *
 * Revision 1.2  2006/11/14 13:33:45  a0190596
 * no message
 *
 * Revision 1.1  2006/04/25 15:00:39  a0190596
 * Initial version in CVS.
 *
 *
 *
 *
 *
 ******************************************************************************************************/
