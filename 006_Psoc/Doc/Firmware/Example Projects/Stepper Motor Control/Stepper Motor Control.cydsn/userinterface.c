/*******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: userinterface.c
* Version			: 1.0
* Device Used		: CY8C4245AXI-483     
* Software Used		: PSoC Creator 4.2
* Compiler Used		: ARM GCC 5.4.1 
* Related Hardware  : CY8CKIT-042 PSoC 4 Pioneer Kit + CY8CKIT-037 PSoC 4
*                     Motor Control Evaluation Kit + 42-mm Stepper Motor
*                     (42BYGH403AA)
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
#include "UI_paras.h"
#include "stepper.h"
#include "userinterface.h"

uint8 s2State = FALSE;
uint8 s2Pressed = FALSE;
uint8 s2DebounceCount = FALSE;

static uint8 bcpTxBuffer[32];

extern uint16 curRpm;

extern STEPPER Sm;

/*******************************************************************************
* Function Name: InterfaceProcess
********************************************************************************
*
* Summary:
* This function is to detect whether the start-stop switch was pressed. If pressed, set 
* the variable "s2Pressed" to True, and it'll be handled in main function
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void InterfaceProcess(void)
{
	
	/* Read and Debounce switches */		
	if(!s2State)						/* If Previous Value is Low,Switch is Pressed*/
	{
		if(!SW2_Read())				/* If current Value is Low, Switch is Pressed */
		{
			s2DebounceCount++;
			if(s2DebounceCount > GlitcheFltCnt)		/* Eliminate the Glitches */
			{
				s2DebounceCount = 0;
				s2Pressed = TRUE;
				s2State = TRUE;
			}
		}
	}
	else
	{
        /* If Previous Value is Low,Switch is being Pressed; and if Current Value is High, Switch is Reset */
		if(SW2_Read())				
		{
			s2DebounceCount++;
			if(s2DebounceCount > GlitcheFltCnt)
			{
				s2DebounceCount = 0;
				s2State = FALSE;
			}
		}
	}
	
}

/*******************************************************************************
* Function Name: ReadVolt
********************************************************************************
* Summary:
* The ReadVolt function samples Vbus to protect board from low-voltage and high
* voltage. 
*
* Parameters: None 
*
* Return: None
*
*******************************************************************************/

void ReadVolt(void)
{
    uint16 adcResult = 0;	
    static uint8 highVoltCounter=0;
    static uint8 lowVoltCounter=0;    
	adcResult = (ADC_SAR_Seq_1_GetResult16(1) & 0x0FFF);       /* filter unwanted MSB */
    
   /*Over voltage or under voltage*/
    if(adcResult < LVTHRESHOLD)
    {
    	lowVoltCounter++;					/*Record low voltage count*/
    }
    else if (adcResult > HVTHRESHOLD)
    {
    	highVoltCounter++;					/*Record high voltage count*/
    }	   
    else if((lowVoltCounter>0)||(highVoltCounter>0))		/*Eliminate pulse high or low voltage count*/
    {
    	lowVoltCounter--;
		highVoltCounter--;   
    }
	
   /*Last more than voltage error time threshold, then stop motor*/
   	if(lowVoltCounter > 2)
    {       
   		errorFlag = lowVolt;		
    }
    if(highVoltCounter > 2)
    {
   		errorFlag = highVolt;		
    }
}


/****************************************************************************************
*                                                                             
*   Function:    BCPPoll                                                        
*                                                                             
*   Description: BCPPoll routine                                                   
*             Format:  RX8 [h=55] @0speed @1speed [t=AA]                      
*   Parameters:   None                                                          
*                                                                                 
*   Returns:     None                                                                 
*                                                                                   
****************************************************************************************/

void BCPPoll(void)
{
    uint8 index = 0;
    if(UART_BCP_SpiUartGetTxBufferSize())
       return;
    
    bcpTxBuffer[index++] = 0x55;  

    /* speed reference */
    bcpTxBuffer[index++] = (uint8)((curRpm & 0xFF00) >> 8);
    bcpTxBuffer[index++] = (uint8)(curRpm & 0x00FF);       
	
    bcpTxBuffer[index++] = 0xAA;
    UART_BCP_SpiUartPutArray(bcpTxBuffer, index);
}

/*End of File */

