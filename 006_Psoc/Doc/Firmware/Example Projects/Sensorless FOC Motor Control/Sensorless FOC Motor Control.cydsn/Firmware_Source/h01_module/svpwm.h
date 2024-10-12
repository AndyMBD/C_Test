/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: svpwm.h
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

#ifndef __SVPWM_H__
#define __SVPWM_H__

#ifdef __cplusplus
extern  "C" 
{
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
; Define the structure used to initialize the modulator
;----------------------------------------------------------------------------*/
typedef struct stc_SvpwmCfg{
    uint16_t u16Cycle;   // +0, cycle of SVPWM
    uint16_t u16DutyMax; // +2, maximum duty of SVPWM
    uint16_t u16DpwmEn;  // +4, Enable or disable DPWM function
}stc_svm_cfg_t;

/*-----------------------------------------------------------------------------
; Define the operation structure for modulator
;----------------------------------------------------------------------------*/
typedef struct stc_SvpwmCalc{
    int32_t  i32QN_VaIn;   // +0, Alpha component of reference voltage vector
    int32_t  i32QN_VbIn;   // +4, Beta component of reference voltage vector
    int32_t  i32QN_VdcIn;  // +8, DC-Link voltage
    int32_t  i32Q30_DutyK0;//+12, The ratio of the maximum duty and cycle
    uint16_t u16Cylce;     //+16, Cycle of SVPWM
    uint16_t u16DutyMax;   //+18, Maximum duty of SVPWM  
    int32_t  i32QN_VaReal; //+20, Alpha component of compensated voltage vector
    int32_t  i32QN_VbReal; //+24, Beta component of compensated voltage vector
    uint16_t u16T1;        //+28, Duration of the first active vector
    uint16_t u16T2;        //+30, Duration of the second active vector
    uint16_t u16Uon;       //+32, M1arison value for phase a
    uint16_t u16Von;       //+34, M1arison value for phase b
    uint16_t u16Won;       //+36, M1arison value for phase c
    uint16_t u16Sector;    //+38, The region number where the real voltage vector local in
    uint16_t u16OvFlag;    //+40, Out of modulation range flag
    uint16_t u16DpwmEn;    //+42, Enable or disable DPWM function 
}stc_svm_calc_t;

typedef struct stc_svm_gen
{
    int16_t    i16DutyMax;
    int16_t    i16Cycle;
    int16_t    i16DeadTime;
    int16_t    i16SampleWind;
    int16_t    i16SampleOffeset;

    int16_t    i16T1;
    int16_t    i16T2;

    int16_t    i16Aon;
    int16_t    i16Bon;
    int16_t    i16Con;

    int16_t    i16AonUp;
    int16_t    i16BonUp;
    int16_t    i16ConUp;
    int16_t    i16AonDown;
    int16_t    i16BonDown;
    int16_t    i16ConDown;

    volatile int16_t i16AdcTrigT2;
    volatile int16_t i16AdcTirgT1;

    volatile int16_t i16AdcTrigIndex;
    volatile int16_t i16AdcTrigT1Buf;

    int8_t i8ShuntNum;
    int8_t i8Sector;
    int8_t i8SectorPre;
}stc_svm_gen_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
extern int8_t Svm_InitCalc(stc_svm_cfg_t *cfg, stc_svm_calc_t *obj);

extern void Svm_Calc(stc_svm_calc_t *obj);
   
#ifdef __cpulspuls
}
#endif

#endif /* __SVPWM_H__ */
