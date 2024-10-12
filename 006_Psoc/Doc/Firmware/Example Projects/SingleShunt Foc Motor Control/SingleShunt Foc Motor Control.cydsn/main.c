/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
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
#include "Cymc.h"
#include "interface.h"
#include "foc.h"

int main()
{
    /* Opamp Init */
	Opamp_A_Start();
	
	/* SAR ADC Init */
	Cymc_HAL_ADCStart();
	ISR_ADC_StartEx(ISR_SARADC);
	
    /* TCPWM Init */
	Cymc_HAL_PWMOutputDisable();
    Cymc_HAL_PWMStart();
	
	/* PWM Main ISR Init    */
    PWM_MainLoop_ISR_StartEx(FOC_MainLoop_ISR);
	ADC_Timer_Start();
	
	/* Lpcomp and IDAC Init For Bus Current Protection */
	LPComp_IbusPt_Start();
	IDAC_IbusPt_Start();
	IDAC_IbusPt_SetValue(0xFA);
	isr_Ibus_Over_StartEx(Ibus_Over_ISR);

	
    /* Motor Parameters Init */
	Cymc_MAL_MotorInit();
	
	/*UART Init*/
	UART_BCP_Start();
	/* Enable global interrupts. */
    CyGlobalIntEnable;                  
       
    for(;;)
    {  
        key_poll();
		
        if(bIsStart == START_MOTOR)
        {
            if(motor.runState == M_STOP && errState == 0)
            {
                Cymc_MAL_MotorStart();
            }
            bIsStart = IDLE;
        }
        else if(bIsStart == STOP_MOTOR)
        {
			Cymc_MAL_MotorStop();
            bIsStart = IDLE;
        }
		else 
		{
			if(motor.runState != M_STOP)
			{
				/* only send data out when motor is running */
				CyDelay(1);
	            BCPPoll();
				LED2_Write(0);
			}
			else
			{
				if(errState == 0)
				{
					LED2_Write(1);
				}
				else
				{
					if(secFlag)
					{
						secFlag = 0;
						LED2_Write((~LED2_Read())&1);
					}
				}
			}
		}
    }
}

/* [] END OF FILE */
