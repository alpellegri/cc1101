;; DUMMY APPLICATION DESCRIPTOR

#define ASM_FILE
#include "app_descriptor.h"


;;-------------------------------------------------------------------------------------------------------
appDescData     SEGMENT CODE;
                RSEG    appDescData;

;; Dummy function which is called by the bootloader instead of chip detection and USB event handler
DummyFunc:
                RET;

;;  Application descriptor + USB configuration, interface and endpoint descriptors
		        ; SmartRF04EB application descriptor
                PUBLIC appDesc;
appDesc:        DW 0DDDDH;              ; Firmware ID (0400 for the standard SmartRF04EB firmware)
                DW 0DDDDH;              ; Firmware revision
                DW DummyFunc;           ; Pointer to the chip detection function
                DW DummyFunc;           ; Pointer common USB interrupt handler
                DW DummyFunc;           ; Pointer to the USB vendor request handler
                DW DummyFunc;           ; Pointer to the USB IN endpoint handler
                DW DummyFunc;           ; Pointer to the chip detection function

usbConfigDesc:  ; USB configuration descriptor
                DB 09H;                 ; Length
                DB 02H;                 ; Type
                DW 1200H;               ; Totallength
                DB 01H;                 ; NumInterfaces
                DB 01H;                 ; bConfigurationValue
                DB 00H;                 ; iConfiguration
                DB 40H;                 ; bmAttributes
                DB 50;                  ; MaxPower

                ; USB interface descriptor
usbIfDesc:      DB 09H;                 ; bLength
                DB 04H;                 ; bDescriptorType
                DB 00H;                 ; bInterfaceNumber
                DB 00H;                 ; bAlternateSetting
                DB 00H;                 ; bNumEndpoints
                DB 0FFH;                ; bInterfaceClass
                DB 0FFH;                ; bInterfaceSubClass
                DB 0FFH;                ; bInterfaceProcotol
                DB 00H;                 ; iInterface
appDescEnd:
;;-------------------------------------------------------------------------------------------------------


;;-------------------------------------------------------------------------------------------------------
;;  Magic word used by the bootloader
                CSEG  AT      846H;
                DB MAGIC_WORD_0;
                DB MAGIC_WORD_1;
                DB MAGIC_WORD_2;
                DW appDesc;
;;-------------------------------------------------------------------------------------------------------


                END;
