/**************************************************************************/ /**
     @file       file name

     @brief      describtion

 ******************************************************************************/
#ifndef COMMON_H
#define COMMON_H

#ifdef __cplusplus
extern "C" {
#endif

//-------------------------------------------------------------------------------------------------------
typedef unsigned char BOOL;

// Data
typedef unsigned char BYTE;
typedef unsigned short WORD;
typedef unsigned long DWORD;

// Unsigned numbers
typedef unsigned char UINT8;
typedef unsigned short UINT16;
typedef unsigned long UINT32;

// Signed numbers
typedef signed char INT8;
typedef signed short INT16;
typedef signed long INT32;
//-------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------
// Common values
#ifndef FALSE
#define FALSE 0
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef NULL
#define NULL 0
#endif

#ifdef __cplusplus
}
#endif

#endif
