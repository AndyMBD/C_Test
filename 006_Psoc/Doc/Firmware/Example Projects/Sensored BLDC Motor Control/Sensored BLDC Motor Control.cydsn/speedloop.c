/*******************************************************************************
* Project Name		: Sensored BLDC Motor Control
* File Name			: speedloop.c
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
#include "speed.h"
#include "motor.h"

int32 piOut = INIT_PIOUT;       /* 60*(1<<16)PI loop output value */

/*******************************************************************************
* Function Name: SpeedPID
********************************************************************************
* Summary:
* The SpeedPID function implements PID regulator for motor speed closed loop control.. 
*
* Parameters: None
*    
* Return: None
*
*******************************************************************************/
void SpeedPID(void)
{
    int16 speedErr;
    static int16 speedErrPrev = 0x00;
    int16 speedErr2;
    int32 result;
    uint16 kp,ki;
  
    if(speedRef > MIDDLE_SPEED_CMD) 		/* Low Speed, Low Coefficient  */
    {
        kp = UI_Data.kp;
        ki = UI_Data.ki;
    }
    else									/* High Speed, High Coefficient  */
    {
        kp = UI_Data.kp << 1;
        ki = UI_Data.ki << 1;
    }
   
    speedErr = speedCur - speedRef; 
    
    /* Calculate output of integration */	
    result = (int32 )(speedErr * ki);		
	piOut += result;
	
    /* Calculate output of proportional */		
	speedErr2 = (speedErr-speedErrPrev);			
	speedErrPrev = speedErr;
	
	result = (int32)(speedErr2 * kp);
	piOut += result;
	

	if(piOut>PIOUT_MAX)
		piOut= PIOUT_MAX;
    if(piOut<PIOUT_MIN)
		piOut= PIOUT_MIN;
				
	dutyCycle = piOut>>16;  

	PWM_Drive_WriteCompare(dutyCycle);
}
/* [] END OF FILE */
