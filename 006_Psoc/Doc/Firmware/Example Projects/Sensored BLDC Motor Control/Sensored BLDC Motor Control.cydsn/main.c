/*******************************************************************************
* Project Name		: Sensored BLDC Motor Control
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
#include "motor.h"
#include "speed.h"
#include "userinterface.h"
#include "getvalue.h"


int main()
{  
    uint8 HallReader =0;    
	uint16 pwmCntLocal = 0;
    
    /* initialize parameters in UI FW */
    Init_UI_FW();    
    /* disable global interrupt */ 
	CyGlobalIntDisable;	
    /* initialize system hardware components */	
    Init_HW();   	
    /*Enable Global interrupt*/
	CyGlobalIntEnable;    
    /* initialize UI hardware components */ 
	Init_UI_HW();
    
    /* stop motor as default */
	UI_Cmd.run =  FALSE;     
    /* Set Motor Direction */
    /* CLOCK_WISE = 0x01, and COUNTER_CLOCK_WISE = 0x00 */  
    CtrlReg_Dir_Write(UI_Data.Dir);
    
    for(;;)
    {
        /* Scan sensors and handle event */
	    ButtonProcess();
		
        /* Measure bus voltage */
        VoltageCheck();	
		
        /*Stop motor by disabling PWM output */
		if(UI_Cmd.run == FALSE)
        {
			/* Turning off LED when motor stopping */
			STATUS_LED_Write(1);			
        	CtrlReg_PWMOut_Write(0x00);  
			PWM_Drive_WriteCompare(PWM_Drive_ReadPeriod()>>16);
            
        	piOut = INIT_PIOUT;
        	HallReader = TRUE;
        } 
		
		/* Error Protection: Disable PWM and shining LED at 1Hz */
		if(errorState != 0)
		{
			CtrlReg_PWMOut_Write(0x00);
            
			STATUS_LED_Write(~(STATUS_LED_Read()));
			CyDelay(500);
			STATUS_LED_Write(~(STATUS_LED_Read()));
			CyDelay(500);
		}		
        
		/* Motor Running */
        if(UI_Cmd.run == TRUE && errorState == 0)
        {
			/* Lighting LED when motor running */
			STATUS_LED_Write(0);
			
            /* only send out UART data when motor is running */
		    BCPPoll();
            
            CtrlReg_PWMOut_Write(0x03);  
            
            pwmCntLocal = pwmCnt;
            
            /* Update every 12.5mS*/
           	if((pwmCntLocal & 0xff) == 0xff) 
    	    {
				pwmCntLocal++;
                
                /* Calculate motor speed reference, unit is RPM*/
				UI_Data.speedRpmRef = ReadRpmRef();   
								
                speedRef = (60*FREQ_CAPTURE_CLK)/(uint32)(MOTOR_POLE_PAIRS*UI_Data.speedRpmRef);
    			
                if(UI_Cmd.run == TRUE)
                {
                /*Speed close loop control */            
                	SpeedPID();  
                }	 
    			
                /* Hall Error Detection*/
                HallReader  = Hall_Error_Read();
                if(HallReader == TRUE)
                {               
    				errorState = hallError;
                	UpdateStatusError();
                }			
    		}
            /* Calculate the real time motor speed every 2000 PWM period, unit: RPM*/
            if(pwmCntLocal >= 2000) 
            {
                uint32 tmp1 = 0;
                uint32 tmp2 = 0;
				pwmCntLocal = 0;
                tmp1 = (60*FREQ_CAPTURE_CLK);
                tmp2 = (MOTOR_POLE_PAIRS*speedCur);            
                UI_Data.speedRpm = (tmp1/tmp2);      
            }
            
            pwmCnt = pwmCntLocal;
        }	
    }
}

/* [] END OF FILE */
