/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: field_weaken.h
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

#ifndef __COMP_FW_H__
#define __COMP_FW_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "h03_user\customer_interface.h"
#include "h01_module\filter.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef struct stc_field_weaken
{
    int32_t i32Q20_IdRefMin;
    int32_t i32Q8_IdRefMin;
    int32_t i32Q8_IdRef;
    int32_t i32Q20_IdRef;
    // jason add start
    int32_t Q12_Kp;
    int32_t Q12_Ki;
    int32_t Q8_Error;
    int32_t Q20_Pout;
    int32_t Q20_Iout;
    //jason add end
    int32_t i32Q12_Ki;
    int32_t i32Q20_IdChangeStep;
    stc_one_order_lpf_t stcIdRefLpf;
    uint32_t u32Ts;
    uint32_t u32TsCnt;
    uint8_t u8En;
    uint8_t u8InitialFlag;
} stc_field_weaken_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
extern stc_field_weaken_t      FieldWeaken_Ctrl;

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
extern void MotorFw_FieldWeaken(stc_field_weaken_t * pstcFw,
                 int32_t *pi32_VqMax,
                 int32_t *pi32Q8_IdRef,
                 uint8_t u8RunningStage);

extern void FieldWeaken_Init(uint16_t u16SampleFreq);

#endif /* __COMP_FW_H__ */

