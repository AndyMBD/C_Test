/*******************************************************************************
* Project Name		: Sensorless BLDC Motor Control
* File Name			: main.c
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
#include "BLDCController.h"
#include "Protection.h"
#include "userinterface.h"
#include "Debug.h"

/*******************************************************************************
* Function Name: Main
********************************************************************************
* Summary:
* Main function
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
int main()
{    
    /*Disable Interrupt*/
	CyGlobalIntDisable;     
     
    /* initialize BLDC const parameters */
    BLDC_ParameterInit();
    /* initialize BLDC Controller, default state is motor stop */
    BLDC_ControllerInit();
    /* initialize button processing */
    ButtonInit();
    
    /* Start ADC */
    ADC_Start();
    
    /* initialize status LED */
#if (DEBUG_ZC_DETECTION_OUTPUT_ENABLE)   
    /* enable Status_LED pin output zero-crossing detection signal*/
    CtrlReg_Debug_Control &= ~(DEBUG_STATUS_LED_SEL_MASK);
#else    
    /* set Status LED off */
    CtrlReg_Debug_Control |= (DEBUG_STATUS_LED_MASK);
    CtrlReg_Debug_Control |= (DEBUG_STATUS_LED_SEL_MASK);
#endif  /* DEBUG_ZC_DETECTION_OUTPUT_ENABLE */

#if (DEBUG_GENERAL_TP_OUTPUT_ENABLE)
    CtrlReg_Debug_Control &= ~(DEBUG_GENERAL_TP_MASK);
#endif
    
    /* UART initialization for BCP communication */ 
    UART_BCP_Start();   
 
    /*Enable Interrupt*/
    CyGlobalIntEnable;   
    
    /***************************************************************************
    * Main Loop
    ****************************************************************************/    
    for(;;)
    {   
        /* firmware trigger to start ADC conversion */
        ADC_StartConvert();
        /* check if conversion is completed */
        ADC_IsEndConversion(ADC_WAIT_FOR_RESULT);  
        /* read speed reference from potentiometer */
        ReadSpeedRef();
        /* process button press event */
        ButtonProcess();  
        /* check the over voltage and under voltage error */
        VoltageCheck();
        /* run BLDC based on control flag status */
        BLDC_Run();         
        /* UART Comm to display speed on Cypress Bridge Control Panel */   
        if(BLDC_Control.runFlag == TRUE)
		    BCPPoll();
    }
}

/* [] END OF FILE */
