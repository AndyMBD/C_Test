/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_HAL_ADC.c
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

	

#define __SADC_START(name)       				name##_Start
#define _SADC_START(name)        				__SADC_START(name)
#define SADC_Start()            				_SADC_START(ADC_NAME)()
	
#define __ADC_GetResult16(name)       			name##_GetResult16
#define _ADC_GetResult16(name)        			__ADC_GetResult16(name)
#define SADC_GetResult16(value)            		_ADC_GetResult16(ADC_NAME)(value)
	
#define __ADCCtrlReg_Write(name)       			name##_Write
#define _ADCCtrlReg_Write(name)        			__ADCCtrlReg_Write(name)
#define ADCCtrlReg_Write(value)            		_ADCCtrlReg_Write(ADCREG_NAME)(value)

#define __ADC_SetChanMask(name)       			name##_SetChanMask
#define _ADC_SetChanMask(name)        			__ADC_SetChanMask(name)
#define SADC_SetChanMask(value)            		_ADC_SetChanMask(ADC_NAME)(value)
	
/*ADC Sample Values*/
uint16 adcRawData[CHANNEL_NUM];


/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_ADCStart											  *
*																			  *
*   Description: Init SAR ADC Module					  					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_ADCStart()
{
    /* start SARADC	*/
    SADC_Start(); 
	ADC_SetChanMask(0x01);//
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_ADCReadSample										  *
*																			  *
*   Description: Read ADC Sample Values and Put into wADCRawData			  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_ADCReadSample()
{
    uint8 i = 0;
    for(i = 0; i < CHANNEL_NUM; i++)
    {
        adcRawData[i] = SADC_GetResult16(i) & (0x0FFF);
    }
}
#if SIGNLE_SHUNT
uint16 Cymc_HAL_ADCGetSample16(uint8 channel) 
{
	return (SADC_GetResult16(channel) & (0x0FFF));
}
void Cymc_HAL_ADCEnableCurChannel()
{
	SADC_SetChanMask(0x01);
}
void Cymc_HAL_ADCEnableAllChannel()
{
	SADC_SetChanMask(0x0F);
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_ADCSampleEnable									  *
*																			  *
*   Description: Enable ADC Sample					    					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_ADCSampleEnable()
{
    /* enable GPIO output	*/
    ADCCtrlReg_Write(0x01);       
}
/******************************************************************************
*																			  *
*   Function:    Cymc_HAL_ADCSampleDisable									  *
*																			  *
*   Description: Disable ADC Sample					  					  *
*																			  *				
*   Parameters:   None														  *
*																			  *
*   Returns:     None														  *		
*																			  *		
******************************************************************************/
void Cymc_HAL_ADCSampleDisable()
{
    /* disable GPIO output	*/
    ADCCtrlReg_Write(0x00);
}
#endif
/* [] END OF FILE */
