/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: isr.c
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

#include "h03_user\hardware_config.h"
#include "h03_user\isr.h"

#include "h02_app\adc_sample.h"
#include "define.h"



/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/
int32_t u;
int32_t w;
uint32_t Led_CountTime = 0;

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

/****************************************************************************************
*																			  			*
*   Function:    FOC_MainLoop_ISR								  							*
*																			  			*
*   Description: PWM main ISR, implement base FOC flow											*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
CY_ISR(FOC_MainLoop_ISR)
{
    uint32_t Clear_ISR;

    Clear_ISR = PWM_C_GetInterruptSourceMasked();

	 MotorCtrl_Process();

    PWM_C_ClearInterrupt(Clear_ISR);
}
/****************************************************************************************
*																			  			*
*   Function:    Ibus_Over_ISR								  								*
*																			  			*
*   Description: comparator ISR for bus current over protection										*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
CY_ISR(Ibus_Over_ISR)
{
	LPComp_ClearInterrupt(LPComp_INTR);
	if(LPComp_GetCompare())
	{
		MotorCtrl_stcRunPar.u32ErroType = MOTOR_OVER_CURRENT;
		MotorCtrl_Stop();
		LPComp_ClearInterrupt(LPComp_INTR);
	}
	LPComp_ClearInterrupt(LPComp_INTR);
}

/****************************************************************************************
*																			  			*
*   Function:    AD_Time_ISR								  									*
*																			  			*
*   Description: ADC ISR, sampling ADC value													*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
 CY_ISR(AD_Time_ISR)
{

	 uint32 intr_status;

		/* Read interrupt status register */
        intr_status = ADC_SAR_INTR_REG;
        
        /************************************************************************
        *  Custom Code
        *  - add user ISR code between the following #START and #END tags
        *************************************************************************/
        /* `#START MAIN_ADC_ISR`  */

			Adc_ReadSample();
		
			u = Adc_u16RsltOfAdc[0];
	        w = Adc_u16RsltOfAdc[1];
			MotorCtrl_stcRunPar.i32Q8_Vbus = (Adc_u16RsltOfAdc[SYS_ADC_CH_VDC]* Adc_stcSample.i32Q12_Vbusk)>>6;
			MotorCtrl_stcRunPar.i32Q8_VR = (Adc_u16RsltOfAdc[MOTOR_SPEED_VR])>>5;

			if(MotorCtrl_stcSvmGen.i8ShuntNum >= 2)
	    	{
		        if(Adc_u8MotorStatus == SENSOR_RUNNING)
		        {
		               Adc_stcSample.u8_CompleteFlag = 1;
		        }
		        else if(Adc_u8MotorStatus == SENSOR_STOP)
		        {     
		              Adc_MotorSensorOffesetDetect();            
		        }
	    	}
            Led_CountTime++;
            
        /* `#END`  */
        /* Clear handled interrupt */
        ADC_SAR_INTR_REG = intr_status;
       
}
