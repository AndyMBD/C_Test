/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: adc_sample.h
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

#ifndef __ADC_SAMPLE_H__
#define __ADC_SAMPLE_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "h02_app\motor_ctrl.h"
#include "h03_user\customer_interface.h"
#include "define.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

typedef struct stc_nuti_shunt_offest
{
    int32_t i32Xu;
    int32_t i32Xv;
    int32_t i32Xw;
   uint32_t u32SampleNum;
} stc_muti_shunt_offest_t;

typedef struct
{
    int32_t i32Q12_MotorCurrentFactor;
    volatile uint8_t u8_CompleteFlag;
    int32_t i32Q12_Vbusk;
    int32_t i32Q21_ReciprolVbusk;
    int32_t i32Q12_Vack;
    int32_t i32Q14_Iack;
} stc_sensor_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
extern stc_sensor_t             Adc_stcSample;
extern  uint16_t        Adc_u16RsltOfAdc[];
extern volatile uint8_t         Adc_u8MotorStatus;

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/
extern void Adc_Start(void);

extern void Adc_ReadSample(void);

extern void Adc_MotorCurrentSense(void);

extern void Adc_InitSensorPar(unsigned short Sample_freq);

extern void Adc_MotorSensorOffesetDetect(void);

#endif /* __ADC_SAMPLE_H__ */


