/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: define.h
* Version			: 1.0
* Device Used		: CY8C4245AXI-483     
* Software Used		: PSoC Creator 4.2
* Compiler Used		: ARM GCC 5.4.1 
* Related Hardware  : CY8CKIT-042 PSoC 4 Pioneer Kit + CY8CKIT-037 PSoC 4
*                     Motor Control Evaluation Kit
******************************************************************************
* Copyright (2018), Cypress Semiconductor Corporation. All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* (“Software”), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries (“Cypress”) and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software (“EULA”).
*
* If no EULA applies, Cypress hereby grants you a personal, nonexclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress’s integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, 
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED 
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress 
* reserves the right to make changes to the Software without notice. Cypress 
* does not assume any liability arising out of the application or use of the 
* Software or any product or circuit described in the Software. Cypress does 
* not authorize its products for use in any products where a malfunction or 
* failure of the Cypress product may reasonably be expected to result in 
* significant property damage, injury or death (“High Risk Product”). By 
* including Cypress’s product in a High Risk Product, the manufacturer of such 
* system or application assumes all risk of such use and in doing so agrees to 
* indemnify Cypress against all liability.
*******************************************************************************/

#ifndef G03_Define_H
#define G03_Define_H

#include <stdint.h>
/* header files for Motor Control binary library */
//#include "CymcLib\CymcLib.h"
#include <cytypes.h>

#ifdef  __cplusplus
extern "C" {
#endif

#ifdef  __cplusplus
}
#endif


#define _root
#define __irq

// choose development IDE
#define DEV_ENV_IAR 1
#define DEV_ENV_KEIL 2
#define DEV_ENV DEV_ENV_IAR

// Enable/Disable SH driver
#define EN_DRV_SH 0

#if(DEV_ENV == DEV_ENV_IAR)
#define _interrupt_ _root

#elif(DEV_ENV == DEV_ENV_KEIL)
#define _interrupt_ __irq

#endif

//*****************************************************************************
//
// Define constants
//
//*****************************************************************************



#ifndef CW
#define CW 0
#endif

#ifndef CCW
#define CCW 1
#endif

#ifndef TRUE
#define TRUE 1
#endif

#ifndef FALSE
#define FALSE 0
#endif

#define INVALID_NUM_FP32 -128.0f
#define INVALID_NUM_8BIT 0
#define INVALID_NUM_16BIT 0
#define INVALID_NUM_8BIT_S -128
#define INVALID_NUM_16BIT_S -32768
#define INVALID_STR ""

/******************/
#define BSC_DIV1 0x00
#define BSC_DIV2 0x01
#define BSC_DIV3 0x02
#define BSC_DIV4 0x03
#define BSC_DIV6 0x04
#define BSC_DIV8 0x05
#define BSC_DIV16 0x06



#define APB_DIV1 0x00
#define APB_DIV2 0x01
#define APB_DIV4 0x02
#define APB_DIV8 0x03



/* Qn base value */
#define Q0_BASE  1L
#define Q1_BASE  2L
#define Q2_BASE  4L
#define Q3_BASE  8L
#define Q4_BASE  16L
#define Q5_BASE  32L
#define Q6_BASE  64L
#define Q7_BASE  128L
#define Q8_BASE  256L
#define Q9_BASE  512L
#define Q10_BASE 1024L
#define Q11_BASE 2048L
#define Q12_BASE 4096L
#define Q13_BASE 8192L
#define Q14_BASE 16384L
#define Q15_BASE 32768L
#define Q16_BASE 65536L
#define Q17_BASE 131072L
#define Q18_BASE 262144L
#define Q19_BASE 524288L
#define Q20_BASE 1048576L
#define Q21_BASE 2097152L
#define Q22_BASE 4194304L
#define Q23_BASE 8388608L
#define Q24_BASE 16777216L
#define Q25_BASE 33554432L
#define Q26_BASE 67108864L
#define Q27_BASE 134217728L
#define Q28_BASE 268435456L
#define Q29_BASE 536870912L
#define Q30_BASE 1073741824L
#define Q31_BASE 2147483648L

#define INT8U_MAX 255
#define INT8S_MAX 127
#define INT8S_MIN -127

#define INT16U_MAX 65535
#define INT16S_MAX 32767
#define INT16S_MIN -32767

#define INT32U_MAX 4294967295
#define INT32S_MAX 2147483647
#define INT32S_MIN -2147483647

#define Q15_MAX 2147483615L // 65535.999 * Q15_BASE
#define Q15_MIN -2147483648L // -65536 * Q15_BASE



//*****************************************************************************
//
// Define switch
//
//*****************************************************************************

/* define memory mode for variables */
//#define MEM_RAM volatile /* necessary for enable optimizing by EWARM */
//#define MEM_RAM_SUG volatile /* suggest using volatile */

//*****************************************************************************
//
// Define data types
//
//*****************************************************************************

//
// Define the basic data type
//

#ifndef _STD_TYPE__
#define _STD_TYPE__

typedef signed   char       BOOL;   // Boolean
typedef unsigned char       INT8U;  // Unsigned  8 bit quantity
typedef signed   char       INT8S;  // Signed    8 bit quantity
typedef unsigned short      INT16U; // Unsigned 16 bit quantity
typedef signed   short      INT16S; // Signed   16 bit quantity
typedef unsigned int        INT32U; // Unsigned 32 bit quantity
typedef signed   int        INT32S; // Signed   32 bit quantity
typedef unsigned long       LONG32U;// Unsigned 32 bit quantity
typedef signed   long       LONG32S;// Signed   32 bit quantity
typedef unsigned long long  INT64U; // Unsigned 64 bit quantity
typedef signed   long long  INT64S; // signed   64 bit quantity
typedef float               FP32;   // single precision floating point
typedef double              FP64;   // double precision floating point

#endif

/** single precision floating point number (4 byte) */
typedef float        float32_t;
typedef unsigned char       boolean_t;
typedef char                char_t;
/** double precision floating point number (8 byte) */
typedef double       float64_t;



// Qn format storage type
typedef INT8S  Qn_VAL8;
typedef INT16S Qn_VAL16;
typedef INT32S Qn_VAL32;
typedef INT64S Qn_VAL64;

typedef struct
{
    INT8U bit00 : 1;
    INT8U bit01 : 1;
    INT8U bit02 : 1;
    INT8U bit03 : 1;
    INT8U bit04 : 1;
    INT8U bit05 : 1;
    INT8U bit06 : 1;
    INT8U bit07 : 1;
}_stReg8;

typedef union
{
    INT8U all;
    _stReg8 bit;
}_uniReg8;

typedef struct
{
    INT16U bit00 : 1;
    INT16U bit01 : 1;
    INT16U bit02 : 1;
    INT16U bit03 : 1;
    INT16U bit04 : 1;
    INT16U bit05 : 1;
    INT16U bit06 : 1;
    INT16U bit07 : 1;
    INT16U bit08 : 1;
    INT16U bit09 : 1;
    INT16U bit10 : 1;
    INT16U bit11 : 1;
    INT16U bit12 : 1;
    INT16U bit13 : 1;
    INT16U bit14 : 1;
    INT16U bit15 : 1;
}_stReg16;

typedef union
{
    INT16U all;
    _stReg16 bit;
}_uniReg16;

typedef struct
{
    INT32U bit00 : 1;
    INT32U bit01 : 1;
    INT32U bit02 : 1;
    INT32U bit03 : 1;
    INT32U bit04 : 1;
    INT32U bit05 : 1;
    INT32U bit06 : 1;
    INT32U bit07 : 1;
    INT32U bit08 : 1;
    INT32U bit09 : 1;
    INT32U bit10 : 1;
    INT32U bit11 : 1;
    INT32U bit12 : 1;
    INT32U bit13 : 1;
    INT32U bit14 : 1;
    INT32U bit15 : 1;
    INT32U bit16 : 1;
    INT32U bit17 : 1;
    INT32U bit18 : 1;
    INT32U bit19 : 1;
    INT32U bit20 : 1;
    INT32U bit21 : 1;
    INT32U bit22 : 1;
    INT32U bit23 : 1;
    INT32U bit24 : 1;
    INT32U bit25 : 1;
    INT32U bit26 : 1;
    INT32U bit27 : 1;
    INT32U bit28 : 1;
    INT32U bit29 : 1;
    INT32U bit30 : 1;
    INT32U bit31 : 1;
}_stReg32;

typedef union
{
    INT32U all;
    _stReg32 bit;
}_uniReg32;

//*****************************************************************************
//
// Define global variables
//
//*****************************************************************************

/*
 * Define global variables. Global variables can ONLY be defined for one
 * time in a source file, so we always want to define them in the main.c
 * Defining a macro named DEFINE_GLOBAL_VARS in the main.c to realize only
 * define global variables in main.c.
 */
#ifdef DEFINE_GLOBAL_VARS // Define global variables here

#else // Declare global variables here

#endif

//*****************************************************************************
//
// Define macro functions
//
//*****************************************************************************

#define  Q0(value)  (Qn_VAL32)((value) * 0x00000001)
#define  Q1(value)  (Qn_VAL32)((value) * 0x00000002)
#define  Q2(value)  (Qn_VAL32)((value) * 0x00000004)
#define  Q3(value)  (Qn_VAL32)((value) * 0x00000008)
#define  Q4(value)  (Qn_VAL32)((value) * 0x00000010)
#define  Q5(value)  (Qn_VAL32)((value) * 0x00000020)
#define  Q6(value)  (Qn_VAL32)((value) * 0x00000040)
#define  Q7(value)  (Qn_VAL32)((value) * 0x00000080)
#define  Q8(value)  (Qn_VAL32)((value) * 0x00000100)
#define  Q9(value)  (Qn_VAL32)((value) * 0x00000200)
#define Q10(value)  (Qn_VAL32)((value) * 0x00000400)
#define Q11(value)  (Qn_VAL32)((value) * 0x00000800)
#define Q12(value)  (Qn_VAL32)((value) * 0x00001000)
#define Q13(value)  (Qn_VAL32)((value) * 0x00002000)
#define Q14(value)  (Qn_VAL32)((value) * 0x00004000)
#define Q15(value)  (Qn_VAL32)((value) * 0x00008000)
#define Q16(value)  (Qn_VAL32)((value) * 0x00010000)
#define Q17(value)  (Qn_VAL32)((value) * 0x00020000)
#define Q18(value)  (Qn_VAL32)((value) * 0x00040000)
#define Q19(value)  (Qn_VAL32)((value) * 0x00080000)
#define Q20(value)  (Qn_VAL32)((value) * 0x00100000)
#define Q21(value)  (Qn_VAL32)((value) * 0x00200000)
#define Q22(value)  (Qn_VAL32)((value) * 0x00400000)
#define Q23(value)  (Qn_VAL32)((value) * 0x00800000)
#define Q24(value)  (Qn_VAL32)((value) * 0x01000000)
#define Q25(value)  (Qn_VAL32)((value) * 0x02000000)
#define Q26(value)  (Qn_VAL32)((value) * 0x04000000)
#define Q27(value)  (Qn_VAL32)((value) * 0x08000000)
#define Q28(value)  (Qn_VAL32)((value) * 0x10000000)
#define Q29(value)  (Qn_VAL32)((value) * 0x20000000)
#define Q30(value)  (Qn_VAL32)((value) * 0x40000000)

// wait for a while
#define WAIT_A_WHILE(x) {INT32U Wait = x; while(Wait--);}

// get address
#define HWADDR(x) ((INT32U)(&(x)))

// get address by offset
#define HWADDR_BY_OFFSET(BASE_ADDRESS, OFFSET) ((BASE_ADDRESS) + (OFFSET))

//
// Macros for direct register access.
// Also, __IO_REG8, __IO_REG16, __IO_REG32 in 'io_macro.h' can be used.
//
#define HWREGW(x) (*((volatile INT32U *)(x))) // Macros to directly access 32bit register
#define HWREGH(x) (*((volatile INT16U *)(x))) // Macros to directly access 16bit register
#define HWREGB(x) (*((volatile INT8U *)(x))) // Macros to directly access 8bit register

/* Access data by offset */
#define DATA_ACCESS_BY_OFFSET(DATA_TYPE, BASE_ADDRESS, OFFSET) \
        ((DATA_TYPE *)((BASE_ADDRESS) + (OFFSET)))

/* Access bit */
#define SET_BIT_FIELD(Addr, BitIdx, BitFieldWidth, Val) \
        (*((INT32U*)(Addr)) = ((*((INT32U*)(Addr)) & (~(((1UL << (BitFieldWidth)) - 1) << BitIdx))) | (((Val) & ((1UL << (BitFieldWidth)) - 1)) << (BitIdx))))

#define GET_BIT_FIELD(Addr, BitIdx, BitFieldWidth) \
        ((*((INT32U*)(Addr)) >> (BitIdx)) & ((1UL << (BitFieldWidth)) - 1))

//
// FUNCTION:
// Convert Qn value to Qm
//
// PARAMETERS:
// Qn - Qn value
// n  - available n is [0, 31]
// m  - Qm, available m is [0, 31]
//
// RETURN:
// Qm value (32bit)
//
#define Qnm(Qn, n, m) \
    (((n) > (m)) ? ((Qn) >> ((n) - (m))) : ((Qn) << ((m) - (n))))

// Others
#define GET_MAX2(x,y)       (((x) >= (y)) ? (x) : (y))
#define GET_MIN2(x,y)       (((x) <= (y)) ? (x) : (y))
#define GET_ABS(x)          (((x) >= 0) ? (x) : (-x))
#define LIMIT_VAL_TO_SCOPE(Var,Max,Min) \
        (Var = ((Var) > (Max) ? (Max) : ((Var) < (Min) ? Min : Var)))

#endif /* G03_Define_H */
