/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: interface.c
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
#include "h03_user\interface.h"
#include "h02_app\motor_ctrl.h"
#include "define.h"
#include "h01_module\math.h" 
#include "h02_app\motor_ctrl.h"

static uint8 bcpTxBuffer[32];

int32_t temp_Now;
uint32 temp_Add = 0;  
uint16_t Temp_Speed[64] = {0};
uint32_t temp_OutFilter;
/****************************************************************************************
*																			  			*
*   Function:    BCPPoll								  								*
*																			  			*
*   Description: BCPPoll routine   													    *
*				 Format:  RX8 [h=55] @0speed @1speed @0currentU @1currentU [t=AA]       *				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/

void BCPPoll()
{
	uint8 index = 0;
	int32 temp;
    uint32_t temp_Out;

    /**********filter********/
    temp_Now = MotorCtrl_stcRunPar.i32Q8_EstimWmHzf * 60 >>8;

    for(uint8_t i =63; i>0; i--)
    {
        
        Temp_Speed[i] = Temp_Speed[i - 1];
        temp_Add += Temp_Speed[i];
    }
    Temp_Speed[0] = temp_Now;    
    temp_Add += Temp_Speed[0];
    temp_Out = temp_Add>>6;
    temp_OutFilter = (temp_Out + temp_OutFilter*15)>>4;
    temp_Add = 0;
    
	if(UART_BCP_SpiUartGetTxBufferSize())
		return;
	bcpTxBuffer[index++] = 0x55;
	temp = (uint16_t)(temp_OutFilter);
	bcpTxBuffer[index++] = (temp>>8)&0xFF;
	bcpTxBuffer[index++] = (temp)&0xFF;
	temp = (uint16_t)MotorCtrl_stcRunPar.i32TargetSpeedRpm;
	bcpTxBuffer[index++] = (temp>>8)&0xFF;
	bcpTxBuffer[index++] = (temp)&0xFF;
	bcpTxBuffer[index++] = 0xAA;
	UART_BCP_SpiUartPutArray(bcpTxBuffer, index);
	
}
/* [] END OF FILE */
