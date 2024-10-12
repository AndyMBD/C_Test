/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: protect.h
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

#ifndef __COMP_PROTECT_H__
#define __COMP_PROTECT_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "h02_app\motor_ctrl.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef struct
{
	int32_t Theta_Acc;
	stc_uvw_t   _3sC_Ref;
	int32_t U_Cnt;
	int32_t V_Cnt;
	int32_t W_Cnt;
	int32_t Detect_Time_Num;
	int32_t Detect_Time_Cnt;
	
}stc_losephase1_t;


typedef struct
{
	int32_t  Run_Voltage_Max;
	int32_t  Run_Voltage_Min;
	int32_t  Run_Current_Max;
	int16_t Over_Voltage_Num;
	int16_t Under_Voltage_Num;
	int16_t Over_Current_Num;
	int16_t Over_Voltage_Cnt;
	int16_t Under_Voltage_Cnt;
	int16_t Over_Current_Cnt;
}stc_out_range_t;

typedef struct
{
////////////////////////////
	int32_t Cnt;
	int32_t i32_ErrorCnt;
	int32_t i32_ErrorCnt1;
	int32_t i32_ErrorCntT;
	int32_t i32_DelErrorCnt;
	int32_t i32_DelErrorCnt1;
        int32_t i32_Edf;
        int32_t i32_Eqf;
       int32_t i32Q8_MaxPercent1;
        int32_t i32_MaxErrCnt1;
        int32_t i32_TimeOfStage1;
      int32_t i32Q8_MaxPercent2;
        int32_t i32_MaxErrCnt2;
        int32_t i32_TimeOfStage2;        
        int32_t RptCntOfStage2;
        int32_t RptTimeOfStage2;        
        uint8_t u8_Stage;
        uint8_t u8_Flg;
////////////////////	
}stc_motor_lock1_t;

typedef struct
{
    int32_t i32_ErrorFlagHoldTimeNum;           
    int32_t i32_ErrorFlagHoldTimeCnt;	
}stc_protect_flag_clear_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
//extern stc_losephase0_t        Protector_LosePhasePar;
extern stc_losephase1_t        Protector_NoneLoadPar;
extern stc_out_range_t         Protector_OutRangePar;
extern stc_motor_lock1_t       Protector_LockPar;
//extern stc_current_protect_t   Protector_CurrentPrtPar;
extern stc_protect_flag_clear_t    Protector_FlgClr; 

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
extern uint32_t  MotorProtect_MotorLock(stc_motor_lock1_t *pstcPar);

extern uint32_t  MotorProtect_OutRange(stc_out_range_t * pstcPar, stc_uvw_t *pstcMotor3sCurrent, int32_t i32_Vbus);

extern uint32_t  MotorProtect_LosePhase1(stc_losephase1_t *pstcPar,stc_uvw_t *pstcMotor3sCurrent, stc_dq_t *pstcMotor2rCurrentRef);

extern void Protector_Init(int32_t sample_freq);

#endif /* __COMP_PROTECT_H__ */

