/*******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: main.c
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

#include "stepper.h"
#include "UI_paras.h"
#include "userinterface.h"

#define MICROSTEPCLK 8000000
#define STEPS_PER_REV 200
#define MAX_MICROSTEPS 128
#define RPMCOEF (uint32)(MICROSTEPCLK * 60)/(MAX_MICROSTEPS * STEPS_PER_REV)

void MotorInit(void);

CY_ISR(microstep_isr);
CY_ISR(pwm_isr);

UI_CMD          UI_Cmd;
UI_DATA         UI_Data;

uint8  stepEnable = FALSE;
uint8  s2Cnt = 0;
uint8  firstRun = 0;

uint16 pwmCnt = 0;
uint32 rpmReq = 0;
uint16 curRpm = 0;

Error errorFlag = noError;

/*******************************************************************************
* Function Name: main
********************************************************************************
* Summary: Main Function,Parameter is initialized and main loop is included.
*  
* Parameters: None  
*
* Return: None
*  
*******************************************************************************/
int main()
{
    /* PWM Initialization */
	PWM_B_Start();		
	PWM_A_Start();
    PWM_B_TriggerCommand(0x03, PWM_B_CMD_START);

	PWM_B_WriteCounter(0);	
	PWM_A_WriteCounter(0);	
	
    /* SAR and Opamp Initialization */	
	ADC_SAR_Seq_1_Start();
	Opamp_A_Start();
	Opamp_B_Start();
	
	/* UART Init */
	UART_BCP_Start();
	
    /* ISR Configuration */	
	isr_pwm_Start();
    isr_pwm_StartEx(pwm_isr);
	isr_pwm_Enable();
	
	isr_microstep_Start();
    isr_microstep_StartEx(microstep_isr);
	isr_microstep_Enable();

    /* Motor Initialization */
	 MotorInit();
                
	CyGlobalIntEnable;	 
			
    for(;;)
    {				
		/* Update Period = 2000*PWM_Period = 2000*21.25uS = 42.5mS */
		if(pwmCnt++ >= 1999)			
		{  
			pwmCnt=0;
            
            ReadVolt();
            
			InterfaceProcess();		/* Detecting the Switch Pin Pressed or not*/

			/* Get the Rpm demand from ADC */
			rpmReq = (ADC_SAR_Seq_1_GetResult16(0)& 0x0FFF);
            rpmReq ^= 0x0FFF;       /* Reverse the speed increment direction */
			
			/* Filter the result*/
			if(rpmReq < 250)
			{
				rpmReq = 250;
			}
			
			if(rpmReq > 3750)
			{
				rpmReq = 3750;
			}			
		
			/* Limit the speed when low microstep numbers */          
            switch(UI_Data.microStepPace)
            {
                case 16:
                        rpmReq <<= 2;
                        break;
                case 32:
                        rpmReq <<= 3;
                        break;                
                case 64:
                        rpmReq <<= 4;
                        break;                
                case 128:
			    /* Jump the mechanical resonance points, the value should 
                   be adjusted in real-time to fit different motors */                         
                if(rpmReq > 2000 && rpmReq < 2400)
                        {
                            rpmReq = 2400;    
                        }
                        if(rpmReq > 1000 && rpmReq < 1400)
                        {
                            rpmReq = 1400;    
                        }
                        
                        rpmReq <<= 5;                
                        break;
                default:
                        rpmReq <<= 1;
            }                
			
			Timer_1_WritePeriod(rpmReq);    /* Update RPM demand */
			
			/* Calculate the real time motor speed value */
            curRpm = (uint16)((RPMCOEF * UI_Data.microStepPace) / rpmReq);
            
			/* Swap the Start-Stop Demand State */			
			if(s2Pressed)
			{
				s2Pressed = FALSE;
				
				s2Cnt++;
				s2Cnt &= 0x01;
				
				if(s2Cnt == 1)
				{
					UI_Cmd.run = TRUE;
					firstRun = 1;				
				}
				else
				{
					UI_Cmd.run = FALSE;
				}
			}
		}		
		
		/*Stop motor by disabling PWM output */
		if(UI_Cmd.run == FALSE)	
	    {
			STATUS_LED_Write(1);					/* State LED Display */	
			
	       	stepEnable = FALSE; 
			Timer_1_Stop();
	       	Stopping();
        }	
		
		/* Error Protection: Disabling PWM and blinking LED at 1Hz */
		if(errorFlag != noError)
		{
			Control_Start_Write(0);
			
			STATUS_LED_Write(~(STATUS_LED_Read()));
			CyDelay(500);
			STATUS_LED_Write(~(STATUS_LED_Read()));
			CyDelay(500);			
		}		
		
		/* Motor Running */
        if(UI_Cmd.run == TRUE && errorFlag == noError)						/* Start Motor */
        {
          	Control_Start_Write(1);	
			STATUS_LED_Write(0);					/* State LED Display */
			
            /* BCP Comm */
		    BCPPoll();

          	stepEnable = TRUE;
			
			/* Start IDAC and Microstepping Timer at first start up*/
			if(firstRun)
			{
				IDAC_A_Start();
				IDAC_A_SetValue(0);	
				IDAC_B_Start();	
				IDAC_B_SetValue(0);	 
                
				Timer_1_Start();
                                	
				firstRun = 0;    			
    		}			
        }				
    }
}

/*******************************************************************************
* Function Name: microstep_isr
********************************************************************************
*
* Summary:
* This function is Microstepping Timer ISR, it calls the microstepping commutation
* function.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
CY_ISR(microstep_isr)
{
    if(stepEnable)
    {
        Stepping(); /* Go one micro-step*/ 
    }	
    
    Timer_1_ReadStatusRegister();	
}

/*******************************************************************************
* Function Name: pwm_isr
********************************************************************************
*
* Summary:
* This function is PWM ISR when TC happens, it increases PWM ticker and clear flag.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
CY_ISR(pwm_isr)
{
    pwmCnt++;

    PWM_B_ClearInterrupt(PWM_B_INTR_MASK_TC);
}

/*******************************************************************************
* Function Name: MotorInit
********************************************************************************
*
* Summary:
* This function is to initialize motor state.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void MotorInit(void)
{
    Control_Start_Write(0);
	UI_Cmd.run = FALSE;
    UI_Data.microStepPace = 8;		 /* Set the number of micro-step pace: microstep number = 128 / UI_Data.microStepPace */	
	UI_Data.dir = CW;        /* Set rotating direction of motor */	
}

/* [] END OF FILE */
