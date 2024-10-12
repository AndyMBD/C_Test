/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: observer_pll.h
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

#ifndef __COMP_PLL_H__
#define __COMP_PLL_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "h01_module\filter.h"
#include "h01_module\coordinate_transform.h"
/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/


/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef struct stc_ebemf_pll
{
    int32_t i32Q8_Ealpha;
    int32_t i32Q8_Ebeta;
    int32_t i32Q8_Ed;
    int32_t i32Q8_Eq;
    int32_t i32Q8_AlphaCurrentpPre;
    int32_t i32Q8_BetaCurrentPre;
    int32_t i32Q8_AlphaVoltPre;
    int32_t i32Q8_BetaVoltPre;
    int32_t i32Q8_DiDtMax;
    int32_t i32Q8_DiDtMin;

    stc_one_order_lpf_t stcEdLpf;
    stc_one_order_lpf_t stcEqLpf;

    int32_t i32Q8_Edf;
    int32_t i32Q8_Eqf;
    int32_t i32Q8_Edlf;
    int32_t i32Q12_Cos;
    int32_t i32Q12_Sin;
    int32_t i32Q12_CosPre;
    int32_t i32Q12_SinPre;

    int32_t i32Q8_Res;
    int32_t i32Q8_LsDt;
    int32_t i32Q8_Lddt;
    int32_t i32Q8_Lqdt;
    int32_t i32Q12_LdLq;
    int32_t i32Q12_RecipKe;
	int32_t i32Q8_Kp;
    int32_t i32Q22_DeltaThetaKTs;
    int32_t i32Q22_DeltaThetaTs;

    uint16_t  u16KeChgEn;
    uint16_t  u16KeChgCnt;
    int32_t i32Q22_ElecAngle;
    int32_t i32Q8_EstimWmHz;
    int32_t i32Q8_EstimWmHzf;

    stc_one_order_lpf_t stcWmLpf;
} stc_ebemf_pll_t;


typedef struct 
{
    int32_t Pll_Iout;
    int32_t Pll_Q12Ki;
    int32_t Pll_IoutTemp;
}stc_pll_pi_t;
/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
extern stc_ebemf_pll_t Motor_stcPll;
extern stc_pll_pi_t stcPll_Pi;
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

extern void MotorPll_PostionEstimate(stc_ebemf_pll_t *EstimPar,
                      stc_ab_t *Motor_2sVoltage,
                      stc_ab_t *Motor_2sCurrent);
extern void MotorCtrl_InitPll(uint16_t u16SampleFreq);

#endif /* __COMP_PLL_H__ */

