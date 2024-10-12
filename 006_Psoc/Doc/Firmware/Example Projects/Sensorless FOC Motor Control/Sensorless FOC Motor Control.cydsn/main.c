/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: main.c
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

#include <project.h>
#include "define.h"
#include "h03_user\isr.h"
#include "h02_app\motor_ctrl.h"
#include "h03_user\chip_init.h"
#include "h02_app\speed_set.h"
#include "h03_user\interface.h"
#include "stdlib.h"

CY_ISR_PROTO(Ibus_Over_ISR);
CY_ISR_PROTO(AD_Time_ISR);
CY_ISR_PROTO(FOC_MainLoop_ISR);


int main()
{
	/*====================================================================*
	* motor initialization                                               *
	*====================================================================*/
    /* Opamp Init */
	Opamp_A_Start();
	Opamp_B_Start();
	
	/* SAR ADC Init */
	Adc_Start();
	
    /* TCPWM Init */
    PWM_Start();
    /* Motor Parameters Init */
	MotorCtrl_Init(Motor_u16CarrierFreq);
    /* ADC ISR Init    */
    ADC_IRQ_StartEx(AD_Time_ISR);
	/* PWM Main ISR Init    */
    PWM_MainLoop_ISR_StartEx(FOC_MainLoop_ISR);

	/*====================================================================*
     * protection initialization                                          *
     *====================================================================*/
	/* Lpcomp and IDAC Init For Bus Current Protection */
	LPComp_Start();
	IDAC_IbusPt_Start();
	IDAC_IbusPt_SetValue(0x80);//0xC2
	isr_Ibus_Over_StartEx(Ibus_Over_ISR);   
	/*UART Init*/
	UART_BCP_Start();
    /* delay for a while to make DCbus voltage stable, otherwise it triggers under voltage fault */
    CyDelay(100);
	/* Enable global interrupts. */
    CyGlobalIntEnable;  

	
    while(1)
    { 

		if ((MotorCtrl_stcRunPar.u8Status == 0) &&
		    (MotorCtrl_stcRunPar.u32ErroType == 0))
		{            
		    if(MotorCtrl_stcRunPar.i32TargetSpeedRpm != 0)
		    {
	            MotorCtrl_Start(Motor_u16CarrierFreq); 
		    }
		}

		if(MotorCtrl_stcRunPar.u8Status == 1)
		{
		    if(MotorCtrl_stcRunPar.u32ErroType != 0||MotorCtrl_stcRunPar.i32TargetSpeedRpm == 0)
		    {
		        MotorCtrl_Stop(); 
		    }
		} 

		///////////////////set speed///////////////////////
		if(Motor_u8SpeedSetEn == 1)
		{
			AD_Speed();
		}
		//////////////////////run direction and LED/////////////
		Peripheral_Function();
		////////only send data out when motor is running //////////
		if(MotorCtrl_stcRunPar.u8Status == 1)
		{						
			BCPPoll();			
		}

	}
}


/* [] END OF FILE */

