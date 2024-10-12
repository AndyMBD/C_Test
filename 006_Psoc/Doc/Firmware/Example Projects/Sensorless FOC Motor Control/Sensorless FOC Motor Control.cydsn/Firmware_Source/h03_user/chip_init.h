/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: chip_init.h
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

#ifndef __CHIP_INIT_H__
#define __CHIP_INIT_H__

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
/* Assign component real name to HAL pre-defined MACRO */
#define PWMU_NAME        PWM_A			/*PWM Component Name of U Phase*/
#define PWMV_NAME        PWM_B			/*PWM Component Name of V Phase*/
#define PWMW_NAME        PWM_C			/*PWM Component Name of W Phase*/
#define PWMREG_NAME		 PWM_Ctrl_Reg	/*PWM Control Register Name*/
#define ADC_NAME		 ADC			/*SAR ADC Component name*/

//*****************************************************************************
//
// Define switch
//
//*****************************************************************************

///////connector of pwm//////////
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

#define __PWMU_WriteCompare(name)       		name##_WriteCompare
#define _PWMU_WriteCompare(name)        		__PWMU_WriteCompare(name)
#define PWMU_WriteCompare(value)            	_PWMU_WriteCompare(PWMU_NAME)(value)
#define PWMV_WriteCompare(value)            	_PWMU_WriteCompare(PWMV_NAME)(value)
#define PWMW_WriteCompare(value)            	_PWMU_WriteCompare(PWMW_NAME)(value)	

#define __PWMU_ReadPeriod(name)       			name##_ReadPeriod
#define _PWMU_ReadPeriod(name)        			__PWMU_ReadPeriod(name)
#define PWMU_ReadPeriod()            			_PWMU_ReadPeriod(PWMU_NAME)()
#define PWMV_ReadPeriod()            			_PWMU_ReadPeriod(PWMV_NAME)()
#define PWMW_ReadPeriod()            			_PWMU_ReadPeriod(PWMW_NAME)()	

#define __PWMU_WritePeriod(name)       			name##_WritePeriod
#define _PWMU_WritePeriod(name)        			__PWMU_WritePeriod(name)
#define PWMU_WritePeriod(value)            			_PWMU_WritePeriod(PWMU_NAME)(value)
#define PWMV_WritePeriod(value)            			_PWMU_WritePeriod(PWMV_NAME)(value)
#define PWMW_WritePeriod(value)            			_PWMU_WritePeriod(PWMW_NAME)(value)

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

#define __PWMCtrlReg_Read(name)       			name##_Read
#define _PWMCtrlReg_Read(name)        			__PWMCtrlReg_Read(name)
#define PWMCtrlReg_Read            				_PWMCtrlReg_Read(PWMREG_NAME)


#define __PWMU_SetPWMDeadTime(name)       			name##_SetPWMDeadTime
#define _PWMU_SetPWMDeadTime(name)        			__PWMU_SetPWMDeadTime(name)
#define PWMU_SetPWMDeadTime(value)            			_PWMU_SetPWMDeadTime(PWMU_NAME)(value)
#define PWMV_SetPWMDeadTime(value)            			_PWMU_SetPWMDeadTime(PWMV_NAME)(value)
#define PWMW_SetPWMDeadTime(value)            			_PWMU_SetPWMDeadTime(PWMW_NAME)(value)

///////connector of ad//////////
#define __SADC_START(name)       				name##_Start
#define _SADC_START(name)        				__SADC_START(name)
#define SADC_Start()            				_SADC_START(ADC_NAME)()
	
#define __ADC_GetResult16(name)       			name##_GetResult16
#define _ADC_GetResult16(name)        			__ADC_GetResult16(name)
#define SADC_GetResult16(value)            		_ADC_GetResult16(ADC_NAME)(value)
////////connector of clock///////
#define PWM_FREQUENCY_48M    48
#define PWM_FREQUENCY        48000000




/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
                          
/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

extern void PWM_Start(void);

extern void PWM_Stop(void);

extern void MotorCtrl_ConfigPwm(void);
/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

#endif /* __CHIP_INIT_H__ */
/* [] END OF FILE */
