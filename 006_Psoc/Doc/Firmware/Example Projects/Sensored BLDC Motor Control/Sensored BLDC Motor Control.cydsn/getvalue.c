/*******************************************************************************
* Project Name		: Sensored BLDC Motor Control
* File Name			: getvalue.c
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
#include "getvalue.h"
#include "userinterface.h"

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

void VoltageCheck(void)
{
    uint16 adcResult = 0;	
    static uint16 highVoltCounter = 0;
    static uint16 lowVoltCounter = 0;    
	adcResult = (ADC_SAR_Seq_1_GetResult16(0) & 0x00ff);       /* filter unwanted MSB, 2011,12,29 */

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
   	if(lowVoltCounter > LVCNTTHRES)
    {       
   		errorState = lowVolt;
    	UpdateStatusError();		
    }
    if(highVoltCounter > HVCNTTHRES)
    {
   		errorState = highVolt;
    	UpdateStatusError();
    }
}

/*******************************************************************************
* Function Name: ReadRpmRef
********************************************************************************
* Summary:
* The ReadRpmRef function reads speed reference value from potentiometer. 
*
* Parameters: None  
*
* Return: uint16  
*
*******************************************************************************/
uint16 ReadRpmRef(void)
{
	uint16 speedSampleCur = (ADC_SAR_Seq_1_GetResult16(SPEED_REF_CHAN) & 0x0FFF); 
    uint16 speedRange = UI_Data.maxSpeedRpm - UI_Data.minSpeedRpm;
    uint16 speedGiven = 0;
    
    if(speedSampleCur > MAX_SPEED_REF_ADC_VALUE)
        speedSampleCur = MAX_SPEED_REF_ADC_VALUE;
    if(speedSampleCur < MIN_SPEED_REF_ADC_VALUE)
        speedSampleCur = MIN_SPEED_REF_ADC_VALUE;
    
    speedSampleCur -= MIN_SPEED_REF_ADC_VALUE;
    speedGiven = UI_Data.minSpeedRpm + ((uint32)(speedSampleCur * speedRange) / 
                                               (MAX_SPEED_REF_ADC_VALUE - MIN_SPEED_REF_ADC_VALUE));
    
    return speedGiven;
}
/* [] END OF FILE */
