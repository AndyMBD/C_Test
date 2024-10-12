/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: motor_startup.h
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

#ifndef COMPRESSOR_STARTUP_H_
#define COMPRESSOR_STARTUP_H_

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include<stdint.h>
#include"define.h"


/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/
#define MOTOR_PREHEAT            0
#define MOTOR_ORIENTATION        1
#define MOTOR_FORCE_STARTUP      2
#define MOTOR_ENTER_CLOSELOOP    3
#define MOTOR_CHANGE_SPEED       4


/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
typedef struct
{
    boolean_t bStartFinish;
    int32_t  i32Q12_Iq;
    uint32_t u32Timer;
    uint32_t u32PreheatTime;
    int32_t  i32Q8_PreheatWmHz;
    int32_t  i32Q12_PreheatIqRef;
    int32_t  i32Q12_PreheatIncIqTs;
    uint32_t u32OrientTime;
    int32_t  i32Q12_OrientIqRef;
    int32_t  i32Q20_OrientIq;
    int32_t  i32Q20_OrientIncIqTs;
	int32_t  i32Q20_ForceIncIqTs;
    int32_t  i32Q12_ForceIqRef;
	int32_t  i32Q20_ForceIq;
    int32_t  i32Q22_ForceWmHz;
    int32_t  i32Q22_ForceWmHzRef;
    int32_t  i32Q22_ForceIncWmTs;
} stc_motor_start_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern stc_motor_start_t    MotorCtrl_stcStartup;


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
void MotorStart_Init(stc_motor_start_t* pstcStart);

boolean_t MotorStartup(stc_motor_start_t* pstcStart);


#endif /* COMPRESSORSTARTUP_H_ */


