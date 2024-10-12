/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: adc_sample.c
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

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <project.h>
#include "h02_app\adc_sample.h"
#include "h03_user\hardware_config.h"
#include "define.h"
#include "h03_user\chip_init.h"
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/
#define ADC_CH_AMOUNT 4

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/
stc_sensor_t                    Adc_stcSample;
volatile uint8_t                Adc_u8MotorStatus = 0;
 uint16_t               Adc_u16RsltOfAdc[ADC_CH_AMOUNT];

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
static  int32_t                 Adc_i32MotorIuvwOffsetMax;
static  int32_t                 Adc_i32MotorIuvwOffsetMin;
static uint32_t                 Adc_u32MotorOffsetCheckDelay = 0;
static stc_muti_shunt_offest_t  Adc_stcMotorOffset;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/
/******************************************************************************
*																			  *
*   Function:    Adc_Start											  			  *
*																			  *
*   Description: Init SAR ADC Module					  					  		  *
*																			  *				
*   Parameters:   None														         *
*																			  *
*   Returns:     None														  	  *		
*																			  *		
******************************************************************************/
void Adc_Start()
{
    /* start SARADC	*/
    SADC_Start();    
}
/******************************************************************************
*																			  *
*   Function:    Adc_ReadSample										  			  *
*																			  *
*   Description: Read ADC Sample Values and Put into wADCRawData			  		  *
*																			  *				
*   Parameters:   None														  	  *
*																			  *
*   Returns:     None														  	  *		
*																			  *		
******************************************************************************/
void Adc_ReadSample()
{
    uint8 i = 0;
    for(i = 0; i < ADC_CH_AMOUNT; i++)
    {
        Adc_u16RsltOfAdc[i] = SADC_GetResult16(i) & (0x0FFF);
    }
}
/****************************************************************************************
*																			  			*
*   Function:    Adc_InitSensorPar								  								*
*																			  			*
*   Description: ADC parameters init															*
*																			  			*				
*   Parameters:   u16SampleFreq																*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
void Adc_InitSensorPar(uint16_t u16SampleFreq)
{
    Adc_stcMotorOffset.u32SampleNum = 0;
    Adc_stcMotorOffset.i32Xu = 0;
    Adc_stcMotorOffset.i32Xv = 0;
    Adc_stcMotorOffset.i32Xw = 0;
    Adc_i32MotorIuvwOffsetMax = Motor_i32IuvwOffsetNormal
                            + Motor_i32IuvwOffsetRange;
    Adc_i32MotorIuvwOffsetMin = Motor_i32IuvwOffsetNormal
                            - Motor_i32IuvwOffsetRange;
  

    Adc_stcSample.i32Q12_MotorCurrentFactor
            = Q14(ADC_VOLT_REF / (Motor_f32IuvwSampleResistor
            * Motor_i32IuvwAmplifierFactor * ADC_VALUE_MAX));
    Adc_u32MotorOffsetCheckDelay = (signed long) (u16SampleFreq * 3.0);


    Adc_stcSample.u8_CompleteFlag = 0;
    Adc_stcSample.i32Q12_Vbusk       = Q14(ADC_VOLT_REF * SYS_VDC_FACTOR / ADC_VALUE_MAX);
    Adc_stcSample.i32Q21_ReciprolVbusk = Q19(ADC_VALUE_MAX / (SYS_VDC_FACTOR * ADC_VOLT_REF));
}

/****************************************************************************************
*																			  			*
*   Function:    Adc_MotorSensorOffesetDetect								  					*
*																			  			*
*   Description: Motor sensor offset detect														*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
void Adc_MotorSensorOffesetDetect(void)
{
    if(Adc_u32MotorOffsetCheckDelay < 1)
    {     

#if(2 == MOTOR_SHUNT_NUM)
        Adc_stcMotorOffset.i32Xu += Adc_u16RsltOfAdc[COMP_ADC_CH_IU];
        Adc_stcMotorOffset.i32Xw += Adc_u16RsltOfAdc[COMP_ADC_CH_IW];
        Adc_stcMotorOffset.u32SampleNum++;
        if(Adc_stcMotorOffset.u32SampleNum >= Motor_u32IuvwOffsetCheckTimes)
        {
            Adc_stcMotorOffset.i32Xu >>= 6;
            Adc_stcMotorOffset.i32Xw >>= 6;
            Adc_stcMotorOffset.u32SampleNum = 0;
            if(Adc_stcMotorOffset.i32Xu < Adc_i32MotorIuvwOffsetMin || Adc_stcMotorOffset.i32Xu > Adc_i32MotorIuvwOffsetMax)
            {
                MotorCtrl_stcRunPar.u32ErroType |= AD_MIDDLE_ERROR;
            }
            else if(Adc_stcMotorOffset.i32Xw < Adc_i32MotorIuvwOffsetMin || Adc_stcMotorOffset.i32Xw > Adc_i32MotorIuvwOffsetMax)
            {
                MotorCtrl_stcRunPar.u32ErroType |= AD_MIDDLE_ERROR;
            }
            else
            {
                Adc_u8MotorStatus = SENSOR_RUNNING;
            }
        }

#else
#error "The number of shunt resistors used to sense phase current is not configured \
correctly in file 'hardware_config.h'!"
#endif
    }
    else
    {
        Adc_u32MotorOffsetCheckDelay--;
    }
}

/****************************************************************************************
*																			  			*
*   Function:    Adc_MotorCurrentSense								  						*
*																			  			*
*   Description: Motor phase current calculate													*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
void Adc_MotorCurrentSense(void)
{

#if(2 == MOTOR_SHUNT_NUM)

if(Motor_u8RunningDirection == CW)
{
        MotorCtrl_stcIuvwSensed.i32Q8_Xu
            = ((Adc_stcMotorOffset.i32Xu - Adc_u16RsltOfAdc[COMP_ADC_CH_IU])
            * Adc_stcSample.i32Q12_MotorCurrentFactor) >> 6;
        MotorCtrl_stcIuvwSensed.i32Q8_Xv
            = -(MotorCtrl_stcIuvwSensed.i32Q8_Xu
                + MotorCtrl_stcIuvwSensed.i32Q8_Xw);
        MotorCtrl_stcIuvwSensed.i32Q8_Xw
			 = ((Adc_stcMotorOffset.i32Xw - Adc_u16RsltOfAdc[COMP_ADC_CH_IW])
            * Adc_stcSample.i32Q12_MotorCurrentFactor) >> 6;
}
if(Motor_u8RunningDirection == CCW)
{
	MotorCtrl_stcIuvwSensed.i32Q8_Xv
		= ((Adc_stcMotorOffset.i32Xu - Adc_u16RsltOfAdc[COMP_ADC_CH_IU])
		* Adc_stcSample.i32Q12_MotorCurrentFactor) >> 6;
	MotorCtrl_stcIuvwSensed.i32Q8_Xu
		= -(MotorCtrl_stcIuvwSensed.i32Q8_Xv
                + MotorCtrl_stcIuvwSensed.i32Q8_Xw);
	MotorCtrl_stcIuvwSensed.i32Q8_Xw
		= ((Adc_stcMotorOffset.i32Xw - Adc_u16RsltOfAdc[COMP_ADC_CH_IW])
        * Adc_stcSample.i32Q12_MotorCurrentFactor) >> 6;

}
             
#else
#error "The number of shunt resistors used to sense phase current is not configured \
correctly in file 'hardware_config.h'!"
#endif
}

