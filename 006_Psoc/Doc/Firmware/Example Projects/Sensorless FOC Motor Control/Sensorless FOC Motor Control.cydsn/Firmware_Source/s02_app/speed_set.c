/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: speed_set.c
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
#include "h02_app\speed_set.h"
#include "h02_app\motor_ctrl.h"
#include "define.h"
#include "h03_user\isr.h"


uint8_t Start_Cnt = 0;
uint8_t Protect_Cnt = 0;
/****************************************************************************************
*																			  			*
*   Function:    AD_Speed								  									*
*																			  			*
*   Description: Use AD to set speed															*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                               				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

void AD_Speed()
{

        if(MotorCtrl_stcRunPar.i32Q8_VR >= 12)
		{
		    if(MotorCtrl_stcRunPar.i32TargetSpeedRpm > 0)
		    {
				MotorCtrl_stcRunPar.i32TargetSpeedRpm = (((MotorCtrl_stcRunPar.i32Q8_VR - 12)*Motor_u16SpdMax)>>7) + Motor_u16SpdMin;
		    }
		    else
		    {
		        Start_Cnt++;
		        if(Start_Cnt > 250)
		        {
		           MotorCtrl_stcRunPar.i32TargetSpeedRpm = (((MotorCtrl_stcRunPar.i32Q8_VR - 12)*Motor_u16SpdMax)>>7) + Motor_u16SpdMin;
		           Start_Cnt = 0;
		        }
		    }
		}
		
		if(MotorCtrl_stcRunPar.i32Q8_VR <= 9)
		{
			MotorCtrl_stcRunPar.i32TargetSpeedRpm = 0;
			Start_Cnt = 0;
		}
		
		if((MotorCtrl_stcRunPar.i32Q8_VR >9) && (MotorCtrl_stcRunPar.i32Q8_VR <12))
		{
			Start_Cnt = 0;
		}
    
}
/****************************************************************************************
*																			  			*
*   Function:    Peripheral_Function								  							*
*																			  			*
*   Description: Running direction and led set													*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
void Peripheral_Function()
{
	static uint8_t Motor_RunDir = 0;

	 ////////////////////run direction///////////////////
	 if((Pin_Dir_Read() == 0)&&(Motor_RunDir == 1))
	 {
		 Motor_RunDir = 0;
		 CyDelay(10);
		 if(Pin_Dir_Read() == 0)
		 {
			   if(Motor_u8RunningDirection == CW)
			 {
				 Motor_u8RunningDirection = CCW;
			 }
			 else
			 {
				 Motor_u8RunningDirection = CW;
			 }
			 MotorCtrl_Stop();
			 CyDelay(700);
		 }
	 }
	 if(Pin_Dir_Read() == 1)
	 {
		Motor_RunDir = 1; 
	 }
	//////////////////////Led////////////////////////////
    /* blink LED in 1Hz if error happens */
    if(MotorCtrl_stcRunPar.u32ErroType != 0)
    {
        if(Led_CountTime >= Motor_u16CarrierFreq)
        {
            Led_CountTime = 0;
            Pin_Led_Write(Pin_Led_Read() ^ 0x01);
        }
    }
     //////////////////Clear ErrorType//////////////////
     if((MotorCtrl_stcRunPar.u32ErroType != 0)&&(Protect_Cnt < 3)&&(MotorCtrl_stcRunPar.i32TargetSpeedRpm == 0))
     {
        if((MotorCtrl_stcRunPar.i32Q8_VR != 0)&&(MotorCtrl_stcRunPar.i32Q8_VR <= 9))
        {
         Protect_Cnt++;
         MotorCtrl_stcRunPar.u32ErroType = 0;
        }
     }
}
/* [] END OF FILE */
