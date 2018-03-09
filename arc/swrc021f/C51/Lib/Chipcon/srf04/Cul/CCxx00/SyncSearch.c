/*******************************************************************************************************
 *                                                                                                     *
 *        **********                                                                                   *
 *       ************                                                                                  *
 *      ***        ***                                                                                 *
 *      ***   +++   ***                                                                                *
 *      ***   + +   ***     This function search for a 4 bytes sync word.                              *
 *      ***   +                                                                                        *
 *      ***   + +   ***                                                                                *
 *      ***   +++   ***     SyncSearch.c                                                               *
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
#include <Chipcon\srf04\regssrf04.h>
#include <Chipcon\srf04\culsrf04.h>




//-------------------------------------------------------------------------------------------------------
//  Defines
#define GDO1_PIN        P0_1
#define SERIAL_CLK      GDO1_PIN

#define GDO0_PIN        P0_6
#define SERIAL_DATA     GDO0_PIN
//-------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------
//  void culSyncSearch(UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0)
//
//  DESCRIPTION:
//      This function search for a 4 bytes sync word. 
//
//  ARGUMENTS:
//      UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0
//          The sunc word that is being searched for. sync3 is the most significant byte.
//-------------------------------------------------------------------------------------------------------
void culSyncSearch(UINT8 sync3, UINT8 sync2, UINT8 sync1, UINT8 sync0) {
    UINT8 sync0Received, sync1Received, sync2Received, sync3Received;
    BYTE SyncNOK;

    do {
        sync3Received = sync3Received << 1;

        if((sync2Received & MSB) == 0)
            sync3Received &= 0xFE;
        else
            sync3Received |= 0x01;

        sync2Received = sync2Received << 1;

        if((sync1Received & MSB) == 0)
            sync2Received &= 0xFE;
        else
            sync2Received |= 0x01;

        sync1Received = sync1Received << 1;

        if((sync0Received & MSB) == 0)
            sync1Received &= 0xFE;
        else
            sync1Received |= 0x01;

        while (!SERIAL_CLK);

        sync0Received = sync0Received << 1;

        if(SERIAL_DATA == 0)
            sync0Received &= 0xFE;
        else
            sync0Received |= 0x01;
        while (SERIAL_CLK);

        SyncNOK = ((sync3Received != sync3) || 
                   (sync2Received != sync2) || 
                   (sync1Received != sync1) || 
                   (sync0Received != sync0));
    
    } while(SyncNOK);
}// culSyncSearch




/******************************************************************************************************
 * Revision history:                                                                                  *
 *
 * $Log: SyncSearch.c,v $
 * Revision 1.1  2005/11/09 09:33:07  sna
 * Initial version in CVS
 *
 *
 *
 ******************************************************************************************************/