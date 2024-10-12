/*******************************************************************************
* Project Name		: Sensorless BLDC Motor Control
* File Name			: Control.c
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
#include "Control.h"
#include "BLDCController.h"

/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These variables should be populated to other modules. Header file contains 
 * the extern statement for these variables.
 ******************************************************************************/ 

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macros are only used in this module. These Macros should not be populated
 * to other modules.
 ******************************************************************************/

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These variables are only used in this module. These variables should not be 
 * populated to other modules.
 ******************************************************************************/

/*******************************************************************************
* Function Name: SpeedPID
********************************************************************************
* Summary:
* The SpeedPID function implements PID regulator for closed loop control
* voltage. 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void SpeedPID(void)
{
    static int32 preSpeedErr = 0;
    
    int32 curSpeedErr = speedCurCount - speedRefCount;
    int32 speedErrDelta = 0;
    int32 pidOutput = 0;
    uint16 switchSpeed = 0;    
    uint32 ki = 0;
    uint32 kp = 0;
    
    /* change Ki and Kp per speed */
    switchSpeed = SECOND_PER_MINUTE / BLDC_Config.polePairNumber;
    switchSpeed = switchSpeed * SPEED_MEASURE_CLOCK_FREQ / SWITCH_SPEED_RPM;    

    /* adjust Kp and Ki based on speed */
    if(speedRefCount > switchSpeed) 
    {
        kp = BLDC_Config.kp;
        ki = BLDC_Config.ki;
    }
    else /* High speed, High PI gain */
    {
        kp = BLDC_Config.kp << PID_KP_SCALER;
        ki = BLDC_Config.ki << PID_KI_SCALER;
    }
    
    BLDC_Control.pidSpeedErr = curSpeedErr;
    /* Calculate output of integration	*/
    pidOutput += (int32)curSpeedErr * (int32)ki;
    
    /* calculate delta of speed error */		
	speedErrDelta = (curSpeedErr - preSpeedErr);		
	preSpeedErr = curSpeedErr;    
	/* Calculate output of proportional */
	pidOutput += (int32)speedErrDelta * (int32)kp;
       
    if(pidOutput > (5<<PI_RANGE_SCALE))
        pidOutput = (5<<PI_RANGE_SCALE);
    if(pidOutput < -(5<<PI_RANGE_SCALE))
        pidOutput = -(5<<PI_RANGE_SCALE);
    
    /* update PWM duty cycle with PID output */	
    BLDC_Control.pidOutput += pidOutput;
    
    /* limit the PID output */
    if(BLDC_Control.pidOutput > LIMIT_MAX)
        BLDC_Control.pidOutput = LIMIT_MAX;
    if(BLDC_Control.pidOutput < LIMIT_MIN)
        BLDC_Control.pidOutput = LIMIT_MIN;
}

/*******************************************************************************
* Function Name: CalcPWMDuty
********************************************************************************
* Summary:
* The CalcPWMDuty function calculates the new duty cycle based on PID output
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void CalcPWMDuty(void)
{
    int32 pidOutput = BLDC_Control.pidOutput;
    uint16 dutyLocal = 0;
    
    /* shift negative PID output to all positive range */
    pidOutput += LIMIT_HALF_RANGE;    
    dutyLocal = pidOutput >> PI_RANGE_SCALE;   
    
    dutyCycle = dutyLocal;       
}

/* [] END OF FILE */
