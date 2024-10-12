/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: coordinate_transform.h
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

#ifndef __EQU_TRANSFORM_H__
#define __EQU_TRANSFORM_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <stdint.h>

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef struct
{
    int32_t i32Q8_Xu;
    int32_t i32Q8_Xv;
    int32_t i32Q8_Xw;
}stc_uvw_t;

typedef struct
{
    int32_t i32Q8_Xa;
    int32_t i32Q8_Xb;
}stc_ab_t;

typedef struct
{
    int32_t i32Q8_Xd;
    int32_t i32Q8_Xq;
    int32_t i32Q12_Cos;
	int32_t i32Q12_Sin;
}stc_dq_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
extern void Clark(stc_uvw_t *pstc_uvw, stc_ab_t *pstc_ab);

extern void InvClark(stc_ab_t *pstc_ab, stc_uvw_t *pstc_uvw);

extern void Park(stc_ab_t *pstc_ab, stc_dq_t *pstc_dq);

extern void InvPark(stc_dq_t *pstc_dq, stc_ab_t *pstc_ab);

#endif /* __EQU_TRANSFORM_H__ */

