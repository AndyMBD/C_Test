/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: chip_init.c
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

#include "h03_user\chip_init.h"
#include "define.h"
#include <project.h>
#include "h02_app\motor_ctrl.h"


/******************************************************************************
*																			  *
*   Function:    PWM_Start											  			  *
*																			  *
*   Description: Init three TCPWM modules PWMU PWMV PWMW					  	  *
*																			  *				
*   Parameters:   None														  	  *
*																			  *
*   Returns:     None														  	  *		
*																			  *		
******************************************************************************/
void PWM_Start()
{
    /* start 3 phases PWM	*/
    uint8 enableInterrupts;
	/*GPIO set to low*/
	PWM_Ctrl_Reg_Write(0);
	
    PWM_A_Init();

    PWM_B_Init();

    PWM_C_Init();

    PWM_D_Init();
    /* clear counter value after start PWM	*/
    PWMU_WriteCounter(0);
    PWMV_WriteCounter(0);
    PWMW_WriteCounter(0);
    PWM_D_WriteCounter(0);

	MotorCtrl_ConfigPwm();
    /* disable GPIO output as default, enable later based on firmware	*/
	CyDelay(1);
	
    enableInterrupts = CyEnterCriticalSection();
	
	PWM_A_BLOCK_CONTROL_REG |= (PWM_A_MASK|PWM_B_MASK|PWM_C_MASK|PWM_D_MASK);
	
    CyExitCriticalSection(enableInterrupts);  

    /* synchronized start three PWMs */
	PWMU_TriggerCommand(0x0F, PWM_A_CMD_RELOAD); 

}
/******************************************************************************
*																			  *
*   Function:    PWM_Stop											  			  *
*																			  *
*   Description: Disable three TCPWM modules PWMU PWMV PWMW					  	  *
*																			  *				
*   Parameters:   None														  	  *
*																			  *
*   Returns:     None														  	  *		
*																			  *		
******************************************************************************/
void PWM_Stop()
{

	uint8 enableInterrupts;
		
	PWM_Ctrl_Reg_Write(0);
		
	enableInterrupts = CyEnterCriticalSection();

	PWM_A_BLOCK_CONTROL_REG &= (uint32)~(PWM_A_MASK|PWM_B_MASK|PWM_C_MASK);

	CyExitCriticalSection(enableInterrupts);

}
/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_ConfigPwm								  							*
*																			  			*
*   Description: TCPWM modules parameters init													*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/
void MotorCtrl_ConfigPwm(void)
{

    int32_t temp_period;
    if (Motor_u16CarrierFreq < 5000)       /*min carry frequency limit*/
    {
        Motor_u16CarrierFreq = 5000;
    }
    else if (Motor_u16CarrierFreq > 20000) /*max carry frequency limit*/
    {
        Motor_u16CarrierFreq = 20000;
    }
	
    temp_period = PWM_FREQUENCY/Motor_u16CarrierFreq/2;
    PWMU_WritePeriod(temp_period);
	PWMV_WritePeriod(temp_period);
	PWMW_WritePeriod(temp_period);
    PWM_D_WritePeriod((temp_period << 1) - 1);
	
	if((PWMU_ReadPeriod()!=PWMV_ReadPeriod())||(PWMU_ReadPeriod()!=PWMW_ReadPeriod()))
	{
	    MotorCtrl_stcRunPar.u32ErroType |= UNDEFINED_INT;
        MotorCtrl_Stop();
	}
	
    if (Motor_f32DeadTimeMicroSec < 0.5)   /*dead time min limit unit: us*/
    {
        Motor_f32DeadTimeMicroSec = 0.5;
    }

	PWMU_SetPWMDeadTime((int32_t)(Motor_f32DeadTimeMicroSec * PWM_FREQUENCY_48M));
	PWMV_SetPWMDeadTime((int32_t)(Motor_f32DeadTimeMicroSec * PWM_FREQUENCY_48M));
	PWMW_SetPWMDeadTime((int32_t)(Motor_f32DeadTimeMicroSec * PWM_FREQUENCY_48M));

    PWMU_WriteCompare(temp_period);
    PWMV_WriteCompare(temp_period);
    PWMW_WriteCompare(temp_period);
    PWM_D_WriteCompare((temp_period << 1) - 24);
    
    PWMU_WriteCompareBuf(temp_period);
    PWMV_WriteCompareBuf(temp_period);
    PWMW_WriteCompareBuf(temp_period);
    PWM_D_WriteCompareBuf((temp_period << 1) - 24);
	
	MotorCtrl_stcSvmGen.i16Cycle = PWMU_ReadPeriod();
    MotorCtrl_stcSvmGen.i16DutyMax = Q0(MotorCtrl_stcSvmGen.i16Cycle - (PWM_FREQUENCY_48M * 2.6));
    MotorCtrl_stcSvmGen.i16DeadTime = Q0(Motor_f32DeadTimeMicroSec* PWM_FREQUENCY_48M);

}

/* [] END OF FILE */
