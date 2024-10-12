/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_HAL_PWM.c
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
#include "Cymc.h"

#define __PWMU_START(name)       				name##_Start
#define _PWMU_START(name)        				__PWMU_START(name)
#define PWMU_Start()            				_PWMU_START(PWMU_NAME)()
#define PWMV_Start()            				_PWMU_START(PWMV_NAME)()	
#define PWMW_Start()            				_PWMU_START(PWMW_NAME)()	
	
#define __PWMU_STOP(name)       				name##_Stop
#define _PWMU_STOP(name)        				__PWMU_STOP(name)
#define PWMU_Stop()            					_PWMU_STOP(PWMU_NAME)()
#define PWMV_Stop()            					_PWMU_STOP(PWMV_NAME)()
#define PWMW_Stop()            					_PWMU_STOP(PWMW_NAME)()	

#define __PWMU_SetPWMMode(name)       			name##_SetPWMMode
#define _PWMU_SetPWMMode(name)        			__PWMU_SetPWMMode(name)
#define PWMU_SetPWMMode(value)            		_PWMU_SetPWMMode(PWMU_NAME)(value)
#define PWMV_SetPWMMode(value)            		_PWMU_SetPWMMode(PWMV_NAME)(value)
#define PWMW_SetPWMMode(value)            		_PWMU_SetPWMMode(PWMW_NAME)(value)	

#define __PWMU_UNDERFLOW_SET(name)       		name##_UNDERFLOW_SET
#define _PWMU_UNDERFLOW_SET(name)        		__PWMU_UNDERFLOW_SET(name)
#define PWMU_UNDERFLOW_SET		         		_PWMU_UNDERFLOW_SET(PWMU_NAME)
#define PWMV_UNDERFLOW_SET		         		_PWMU_UNDERFLOW_SET(PWMV_NAME)
#define PWMW_UNDERFLOW_SET		         		_PWMU_UNDERFLOW_SET(PWMW_NAME)	
	
#define __PWMU_UNDERFLOW_CLEAR(name)       		name##_UNDERFLOW_CLEAR
#define _PWMU_UNDERFLOW_CLEAR(name)        		__PWMU_UNDERFLOW_CLEAR(name)
#define PWMU_UNDERFLOW_CLEAR		         	_PWMU_UNDERFLOW_CLEAR(PWMU_NAME)
#define PWMV_UNDERFLOW_CLEAR		         	_PWMU_UNDERFLOW_CLEAR(PWMV_NAME)
#define PWMW_UNDERFLOW_CLEAR		         	_PWMU_UNDERFLOW_CLEAR(PWMW_NAME)	

#define __PWMU_TRIG_CONTROL2_REG(name)       	name##_TRIG_CONTROL2_REG
#define _PWMU_TRIG_CONTROL2_REG(name)        	__PWMU_TRIG_CONTROL2_REG(name)
#define PWMU_TRIG_CONTROL2_REG		         	_PWMU_TRIG_CONTROL2_REG(PWMU_NAME)
#define PWMV_TRIG_CONTROL2_REG		         	_PWMU_TRIG_CONTROL2_REG(PWMV_NAME)
#define PWMW_TRIG_CONTROL2_REG		         	_PWMU_TRIG_CONTROL2_REG(PWMW_NAME)	

#define __PWMU_OVERLOW_NO_CHANGE(name)       	name##_OVERLOW_NO_CHANGE
#define _PWMU_OVERLOW_NO_CHANGE(name)       	__PWMU_OVERLOW_NO_CHANGE(name)
#define PWMU_OVERLOW_NO_CHANGE		         	_PWMU_OVERLOW_NO_CHANGE(PWMU_NAME)
#define PWMV_OVERLOW_NO_CHANGE		         	_PWMU_OVERLOW_NO_CHANGE(PWMV_NAME)
#define PWMW_OVERLOW_NO_CHANGE		         	_PWMU_OVERLOW_NO_CHANGE(PWMW_NAME)	

#define __PWMU_CC_MATCH_INVERT(name)       		name##_CC_MATCH_INVERT
#define _PWMU_CC_MATCH_INVERT(name)       		__PWMU_CC_MATCH_INVERT(name)
#define PWMU_CC_MATCH_INVERT		         	_PWMU_CC_MATCH_INVERT(PWMU_NAME)
#define PWMV_CC_MATCH_INVERT		         	_PWMU_CC_MATCH_INVERT(PWMV_NAME)
#define PWMW_CC_MATCH_INVERT		         	_PWMU_CC_MATCH_INVERT(PWMW_NAME)	
	
#define __PWMU_CMD_START(name)       			name##_CMD_START
#define _PWMU_CMD_START(name)        			__PWMU_CMD_START(name)
#define PWMU_CMD_START		         			_PWMU_CMD_START(PWMU_NAME)
#define PWMV_CMD_START		         			_PWMU_CMD_START(PWMV_NAME)
#define PWMW_CMD_START		         			_PWMU_CMD_START(PWMW_NAME)	
	
#define __PWMU_WriteCounter(name)       		name##_WriteCounter
#define _PWMU_WriteCounter(name)        		__PWMU_WriteCounter(name)
#define PWMU_WriteCounter(value)            	_PWMU_WriteCounter(PWMU_NAME)(value)
#define PWMV_WriteCounter(value)            	_PWMU_WriteCounter(PWMV_NAME)(value)
#define PWMW_WriteCounter(value)            	_PWMU_WriteCounter(PWMW_NAME)(value)	
	
#define __PWMU_WriteCompareBuf(name)       		name##_WriteCompareBuf
#define _PWMU_WriteCompareBuf(name)        		__PWMU_WriteCompareBuf(name)
#define PWMU_WriteCompareBuf(value)            	_PWMU_WriteCompareBuf(PWMU_NAME)(value)
#define PWMV_WriteCompareBuf(value)            	_PWMU_WriteCompareBuf(PWMV_NAME)(value)
#define PWMW_WriteCompareBuf(value)            	_PWMU_WriteCompareBuf(PWMW_NAME)(value)	

#define __PWMU_ReadPeriod(name)       			name##_ReadPeriod
#define _PWMU_ReadPeriod(name)        			__PWMU_ReadPeriod(name)
#define PWMU_ReadPeriod()            			_PWMU_ReadPeriod(PWMU_NAME)()
#define PWMV_ReadPeriod()            			_PWMU_ReadPeriod(PWMV_NAME)()
#define PWMW_ReadPeriod()            			_PWMU_ReadPeriod(PWMW_NAME)()	

#define __PWMU_ReadCompare(name)       			name##_ReadCompare
#define _PWMU_ReadCompare(name)        			__PWMU_ReadCompare(name)
#define PWMU_ReadCompare()            			_PWMU_ReadCompare(PWMU_NAME)()
#define PWMV_ReadCompare()            			_PWMU_ReadCompare(PWMV_NAME)()
#define PWMW_ReadCompare()            			_PWMU_ReadCompare(PWMW_NAME)()	
	
#define __PWMU_TriggerCommand(name)       		name##_TriggerCommand
#define _PWMU_TriggerCommand(name)       		__PWMU_TriggerCommand(name)
#define PWMU_TriggerCommand(value1, value2)   	_PWMU_TriggerCommand(PWMU_NAME)(value1, value2)
	
#define __PWMCtrlReg_Write(name)       			name##_Write
#define _PWMCtrlReg_Write(name)        			__PWMCtrlReg_Write(name)
#define PWMCtrlReg_Write(value)            		_PWMCtrlReg_Write(PWMREG_NAME)(value)

/*PWM Period*/
uint16 pwmPeriod = 0;
/*PWM Compare Values*/
uint16 pwmCmp[3];

/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMStart											  *
*																			  *
*   Description: Init three TCPWM modules PWMU PWMV PWMW					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
#if SIGNLE_SHUNT
void Cymc_HAL_PWMStart()
{
    // start 3 phases PWM
    PWM_A_Start();
	//PWM_U_SetPWMMode(PWM_U_PWM_MODE_CENTER | PWM_U_UNDERFLOW_SET| PWM_U_OVERLOW_NO_CHANGE| PWM_U_CC_MATCH_INVERT);
	PWM_A_SetPWMMode(PWM_A_UNDERFLOW_CLEAR| PWM_A_OVERLOW_NO_CHANGE| PWM_A_CC_MATCH_INVERT);
	PWM_A_SetCounterMode(PWM_A_COUNT_UPDOWN1);
    PWM_B_Start();
	//PWM_V_SetPWMMode(PWM_V_PWM_MODE_CENTER | PWM_V_UNDERFLOW_SET| PWM_V_OVERLOW_NO_CHANGE| PWM_V_CC_MATCH_INVERT);
	PWM_B_SetPWMMode(PWM_B_UNDERFLOW_CLEAR| PWM_B_OVERLOW_NO_CHANGE| PWM_B_CC_MATCH_INVERT);
	PWM_B_SetCounterMode(PWM_B_COUNT_UPDOWN1);
    PWM_C_Start();
	//PWM_W_SetPWMMode(PWM_W_PWM_MODE_CENTER | PWM_W_UNDERFLOW_SET| PWM_W_OVERLOW_NO_CHANGE| PWM_W_CC_MATCH_INVERT);
	PWM_C_SetPWMMode(PWM_C_UNDERFLOW_CLEAR| PWM_C_OVERLOW_NO_CHANGE| PWM_C_CC_MATCH_INVERT);
	PWM_C_SetCounterMode(PWM_C_COUNT_UPDOWN1);
    // clear counter value after start PWM
    PWM_A_WriteCounter(0);
    PWM_B_WriteCounter(0);
    PWM_C_WriteCounter(0);
    
    // get the initial duty cycles
    pwmPeriod = PWM_A_ReadPeriod();
    pwmCmp[0] = PWM_A_ReadCompare();
    pwmCmp[1] = PWM_B_ReadCompare();
    pwmCmp[2] = PWM_C_ReadCompare();
    
    // start clock input to PWMs
    Clock_PWM_Start();    
    // disable GPIO output as default, enable later based on firmware
    PWM_Ctrl_Reg_Write(0x00);
    // synchronized start three PWMs
    PWM_A_TriggerCommand(0x07, PWM_A_CMD_START); 
}

#else	
void Cymc_HAL_PWMStart()
{
    /* start 3 phases PWM	*/
    PWMU_Start();
	PWMU_SetPWMMode(PWMU_UNDERFLOW_SET| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT);
    PWMV_Start();
	PWMV_SetPWMMode(PWMV_UNDERFLOW_SET| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT);
    PWMW_Start();
	PWMW_SetPWMMode(PWMW_UNDERFLOW_SET| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT);
    /* clear counter value after start PWM	*/
    PWMU_WriteCounter(0);
    PWMV_WriteCounter(0);
    PWMW_WriteCounter(0);
    
    /* get the initial duty cycles	*/
    pwmPeriod = PWMU_ReadPeriod();
    pwmCmp[0] = PWMU_ReadCompare();
    pwmCmp[1] = PWMV_ReadCompare();
    pwmCmp[2] = PWMW_ReadCompare();
      
    /* disable GPIO output as default, enable later based on firmware	*/
    PWMCtrlReg_Write(0x00);
    /* synchronized start three PWMs */
    PWMU_TriggerCommand(0x07, PWMU_CMD_START); 
}
#endif
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMStop											  *
*																			  *
*   Description: Disable three TCPWM modules PWMU PWMV PWMW					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMStop()
{
    PWMU_Stop();
    PWMV_Stop();
    PWMW_Stop();
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMOutputEnable									  *
*																			  *
*   Description: Enable PWM Outputs					    					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMOutputEnable()
{
    /* enable GPIO output	*/
    PWMCtrlReg_Write(0x01);       
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMOutputDisable									  *
*																			  *
*   Description: Disable PWM Outputs					  					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_PWMOutputDisable()
{
    /* disable GPIO output	*/
    PWMCtrlReg_Write(0x00);
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_PWMOutputUpdate									  *
*																			  *
*   Description: Update	PWM duty and handle the 0% and 100% cases			  *
*				 															  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
#if SIGNLE_SHUNT
void Cymc_HAL_PWMOutputUpdate(uint16 PWMU, uint16 PWMV, uint16 PWMW)
{
	/* handle the 0% and 100% for PWM-U */
	PWMU_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMU_UNDERFLOW_CLEAR| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT));
	if(PWMU == 0)
	{
		PWMU = pwmPeriod + 1;		
		PWMU_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMU_UNDERFLOW_SET| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT));
	}
	else if(PWMU == pwmPeriod)
	{
		PWMU = pwmPeriod + 1;
	}
	/* handle the 0% and 100% for PWM-V */
	PWMV_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMV_UNDERFLOW_CLEAR| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT));
	if(PWMV == 0)
	{
		PWMV = pwmPeriod + 1;		
		PWMV_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMV_UNDERFLOW_SET| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT));
	}
	else if(PWMV == pwmPeriod)
	{
		PWMV = pwmPeriod + 1;
	}
	
	/* handle the 0% and 100% for PWM-W  */
	PWMW_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMW_UNDERFLOW_CLEAR| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT));
	if(PWMW == 0)
	{
		PWMW = pwmPeriod + 1;		
		PWMW_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMW_UNDERFLOW_SET| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT));
	}
	else if(PWMW == pwmPeriod)
	{
		PWMW = pwmPeriod + 1;
	}
    /* Update new duty value into compare buffer registers	*/
    PWMU_WriteCompareBuf(PWMU);
    PWMV_WriteCompareBuf(PWMV);
    PWMW_WriteCompareBuf(PWMW); 
}	
#else	
inline void Cymc_HAL_PWMOutputUpdate()
{
	/* handle the 0% and 100% for PWM-U */
	PWMU_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMU_UNDERFLOW_SET| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT));
	if(pwmCmp[0] == 0)
	{
		pwmCmp[0] = pwmPeriod + 1;		
		PWMU_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMU_UNDERFLOW_CLEAR| PWMU_OVERLOW_NO_CHANGE| PWMU_CC_MATCH_INVERT));
	}
	else if(pwmCmp[0] == pwmPeriod)
	{
		pwmCmp[0] = pwmPeriod + 1;
	}
	/* handle the 0% and 100% for PWM-V */
	PWMV_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMV_UNDERFLOW_SET| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT));
	if(pwmCmp[1] == 0)
	{
		pwmCmp[1] = pwmPeriod + 1;		
		PWMV_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMV_UNDERFLOW_CLEAR| PWMV_OVERLOW_NO_CHANGE| PWMV_CC_MATCH_INVERT));
	}
	else if(pwmCmp[1] == pwmPeriod)
	{
		pwmCmp[1] = pwmPeriod + 1;
	}
	
	/* handle the 0% and 100% for PWM-W  */
	PWMW_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMW_UNDERFLOW_SET| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT));
	if(pwmCmp[2] == 0)
	{
		pwmCmp[2] = pwmPeriod + 1;		
		PWMW_TRIG_CONTROL2_REG = ((0x3Fu) & (PWMW_UNDERFLOW_CLEAR| PWMW_OVERLOW_NO_CHANGE| PWMW_CC_MATCH_INVERT));
	}
	else if(pwmCmp[2] == pwmPeriod)
	{
		pwmCmp[2] = pwmPeriod + 1;
	}
    /* Update new duty value into compare buffer registers	*/
    PWMU_WriteCompareBuf(pwmCmp[0]);
    PWMV_WriteCompareBuf(pwmCmp[1]);
    PWMW_WriteCompareBuf(pwmCmp[2]); 
}
#endif


/* [] END OF FILE */
