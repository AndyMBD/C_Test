/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
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
#include "Cymc.h"
#include "interface.h"
#include "foc.h"

#define KEY_PRESSED     1
#define KEY_RELEASED    0
#define LEVEL_HIGH      1
#define LEVEL_LOW       0
uint8 bIsStart = 0;	/*motor start/stop variable: 1-start 2-stop*/

static uint8 key[3], keyTemp[3] = {0,0,0},keyState[3] = {0,0,0};
static uint8 bcpTxBuffer[32];
/****************************************************************************************
*																			  			*
*   Function:    key_poll								  								*
*																			  			*
*   Description: key poll routine   													*
*																			  			*				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void key_poll()
{
    key[0] = SW2_Read();
    
    if(keyTemp[0] == LEVEL_LOW && key[0] == LEVEL_HIGH)
    {
        key[0] = SW2_Read();
        keyState[0] = KEY_PRESSED;
        CyDelay(2);
    }
    if(keyTemp[0] == LEVEL_HIGH && key[0] == LEVEL_LOW && keyState[0] == KEY_PRESSED)    
    {
        keyState[0] = KEY_RELEASED;
        if(motor.runState == M_STOP)
        {
            bIsStart = START_MOTOR;
            CyDelay(2);
        }
        else
        {
            bIsStart = STOP_MOTOR;
            CyDelay(2);
        }
    }
    keyTemp[0] = key[0];    
}
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
	if(UART_BCP_SpiUartGetTxBufferSize())
		return;
	bcpTxBuffer[index++] = 0x55;
	temp = motor.speedRPM;
	bcpTxBuffer[index++] = (temp>>8)&0xFF;
	bcpTxBuffer[index++] = (temp)&0xFF;
	temp = Cymc_GFL_Mul_Q15(motor.speedRef,motor.ratedSpeedRPM);
	bcpTxBuffer[index++] = (temp>>8)&0xFF;
	bcpTxBuffer[index++] = (temp)&0xFF;
	bcpTxBuffer[index++] = 0xAA;
	UART_BCP_SpiUartPutArray(bcpTxBuffer, index);
	
}
/* [] END OF FILE */
