/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: hardware_config.h
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

#ifndef __HW_CFG_H__
#define __HW_CFG_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "define.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/******************************************************************************/
/**system clock = CLK_XTAL_FREQ * (CLK_SYS_FREQMULTI + 1)                   ***/
/******************************************************************************/
#define CLK_XTAL_FREQ                 4             // MHz   
#define CLK_SYS_FREQMULTI             48             // System clock frequency multiplication constant
 
#define ADC_VOLT_REF               5.0f     // Reference voltage for ADC
#define ADC_VALUE_MAX              4096.0f     // 12-bits ADC  #define UVW_VOLTAGE_SAMPLE_EN      0

#define COMP_ADC_CH_IU         0      // ADC channel - Iu
#define COMP_ADC_CH_IV         4      // ADC channel - Iv
#define COMP_ADC_CH_IW         1      // ADC channel - Iw
#define SYS_ADC_CH_VDC         2      // ADC channel - Vdc
#define MOTOR_SPEED_VR         3

#define LOW_POLAR            0
#define HIGH_POLAR           1

#define  MOTOR1_PWM_POLAR_AVAILABLE  HIGH_POLAR

#if 0  
#define ADC_CH_VU         1      // ADC channel - Iu
#define ADC_CH_VV         3      // ADC channel - Iv
#define ADC_CH_VW         6      // ADC channel - Iw
#endif   
   
#define SYS_VDC_FACTOR         20.1  //94.0 //107.0
#define MOTOR_SHUNT_NUM           2      // The number of shunt used to sense current
#if 0
#define     QPRC_ZEROMATCH 0       //0:not match, 1:matched
#define     QPRC_SWAPBIT   0
#define     QPRC_ZEROANGLE  1051195;
   
   
#define SPEED_SAMPLE_EN        0
#define MOTOR_ADC_CH_SPEED     9 
#endif

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
                          
/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

#endif /* __HW_CFG_H__ */

