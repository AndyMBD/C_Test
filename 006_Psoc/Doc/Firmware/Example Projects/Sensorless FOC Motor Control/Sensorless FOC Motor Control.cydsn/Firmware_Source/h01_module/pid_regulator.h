/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: pid_regulator.h
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

#ifndef __PID_REG_H__
#define __PID_REG_H__

#ifdef __cplusplus
extern "C"
#endif

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include<stdint.h>


/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*-----------------------------------------------------------------------------
Define the structure for the pid
------------------------------------------------------------------------------*/
typedef struct
{
	int32_t i32Q15_Kp;          // +0
	int32_t i32Q15_Ki;          // +4
	int32_t i32Q15_Kd;          // +8
	int32_t i32QN_OutMax;       //+12
	int32_t i32QN_OutMin;       //+16
	int32_t i32QN_EAbsMax;      //+20
	int32_t i32QN_E1;           //+24
	int32_t i32QN_E2;           //+28
	int32_t i32QN_Out1;         //+32
	int32_t i32QN_Du0;          //+36
}stc_pid_t;

typedef struct
{
    int32_t i32Q15_kp;
    int32_t i32Q15_ki;
    int32_t i32Q15_kd;
    int32_t I_cnt;
    int32_t I_timer;
    int32_t i32_Pout;
    int32_t i32_Iout;
    int32_t i32_Dout;
    int32_t i32_Out;
    int32_t i32_OutPre;
    int32_t i32QN_Iout;
    int32_t i32_OutMax; // output upper limitation
    int32_t i32_OutMin; // output lower limitation    
    int32_t i32_E0;     // e0 limitation
    int32_t i32_E1;
    int32_t i32_E0Max;
    int32_t i32_E0Min;
}stc_pid_cfg_t;



/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

/*-----------------------------------------------------------------------------
; Function name: Pid_Reg0
; Syntax: uint8_t  Pid_Reg0(stc_pid_t* pstcPid, int32_t i32QN_E0);
; Arguments: 
;   @pstcPid,  point to the PID calculation object structure;
;   @i32QN_E0, last error.
; Return: void
; Description:
;   Implement PID regulation.
; History:
;-----------------------------------------------------------------------------
; version   | Date       | Author  |  Modification
;-----------------------------------------------------------------------------
;  V0.1.0   | 2013/12/20 | Stom.Fu |  First edit
;---------------------------------------------------------------------------*/
void Pid_Reg0(stc_pid_t *pstcPid, int32_t i32QN_E0);

void Pid_Pos(stc_pid_cfg_t *pstcVal, int32_t Error);
void Pid_Inc(stc_pid_cfg_t *pstcVal, int32_t Error);
extern void Pid_IqrefLimt(stc_pid_cfg_t *pstcVal, int32_t Error);

#ifdef __cplusplus
}
#endif


#endif /* __PID_REG_H__ */
