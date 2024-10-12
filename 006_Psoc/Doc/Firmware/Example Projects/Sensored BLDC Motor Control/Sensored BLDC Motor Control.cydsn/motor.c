/*******************************************************************************
* Project Name		: Sensored BLDC Motor Control
* File Name			: motor.c
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
#include "userinterface.h"
#include "getvalue.h"

/* UI_MOTOR_STATUS Status */
UI_CMD  UI_Cmd;
UI_DATA UI_Data;

uint16 speedRef = 0;            /* Motor speed Reference */
uint16 speedCur = 3000;         /* Current motor speed. */

uint16 preSpeedCur = 0;
uint16 preCntCaptur = 30000;

uint8  dutyCycle = 0;            /* really now used as BYTE since use 8-bit PWM */
uint16 pwmCnt = 0;
uint8 firstRun = 1;
uint8 ocBlankCnt = 0;
Error_T errorState = no_error;
uint8 stateSys = STATUS_STOP;

/*******************************************************************************
* Function Name: pwm_isr
********************************************************************************
*
* Summary:
* This function is PWM ISR. When TC happens, it increases PWM ticker, clears flag and 
* enables over current protection ISR after startup.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
CY_ISR(pwm_isr)
{	
	pwmCnt++;  
	
    /* Avoid the current pulse interference in motor start-up*/
    if (UI_Cmd.run)
    {
        if(firstRun == 1)
        {
            ocBlankCnt++;
        }
        if (ocBlankCnt > 2)    /*Ignore the first 2 PWM period = 50uS*3=150uS, then enable over current ISR */
        {
            firstRun = 0;
            ocBlankCnt = 0;
            isr_oc_Enable();
            isr_oc_ClearPending();
        }
    }
	
	/* Calculate the real time motor speed every 2000 PWM period*/
	if(pwmCnt >= 2001)
	{
		pwmCnt = 0;
	}

	PWM_Drive_ClearInterrupt(PWM_Drive_INTR_MASK_TC);
}

/*******************************************************************************
* Function Name: speed_measure_isr
********************************************************************************
*
* Summary:
* This function is ISR for motor speed measurement. 
* 
* Parameters: None
*
* Return: None
*
*******************************************************************************/
CY_ISR(speed_measure_isr)
{
    uint16 cntCaptur = 0;
    
    cntCaptur = Counter_Spd_ReadCapture();
	
	speedCur = preCntCaptur - cntCaptur;
	
	/* If speed is too low, Regard motor is stopped*/
     if(speedCur > 5000)		/* < 300Rpm*/
		speedCur = 5000; 
	
    /* filter for speed measured */
    speedCur = (preSpeedCur >> 2) + (preSpeedCur >> 1) + (speedCur >> 2);	
	
    preCntCaptur = cntCaptur;
	preSpeedCur = speedCur; 
	
	Counter_Spd_ClearInterrupt(Counter_Spd_INTR_MASK_CC_MATCH);
}

/*******************************************************************************
* Function Name: over_current_isr
********************************************************************************
*
* Summary:
* This function is ISR for over current protection. It updates the error state flag. 
* 
* Parameters: None
*
* Return: None
*
*******************************************************************************/
CY_ISR(over_current_isr)
{	
    UpdateStatusError();		/* If over current happens, stop motor*/
    errorState = overCur;
    LPComp_OC_ClearInterrupt(LPComp_OC_INTR_RISING);
}

/*******************************************************************************
* Function Name: Init_UI_FW
********************************************************************************
* Summary: This function initializes parameters used in motor running.
*  
* Parameters: None  
*
* Return: None
*  
*******************************************************************************/

void Init_UI_FW(void)
{
    /* Setting UI Initial parameter*/
	UI_Data.Dir = CLOCK_WISE;				
    UI_Data.maxSpeedRpm = 4000;
    UI_Data.minSpeedRpm = 500;
    UI_Data.speedRpmRef = 1000;
    UI_Data.polePairs = 4;
    UI_Data.maxCurr = MAX_CURR_MEDIUM; 
    UI_Data.kp = 500;
    UI_Data.ki = 50;
	
}


/*******************************************************************************
* Function Name: Init_HW
********************************************************************************
* Summary: This function initializes system hardware peripherals 
*  
* Parameters: None  
*
* Return: None
*  
*******************************************************************************/

void Init_HW(void)
{
    /*PWM Initialization*/
	PWM_Drive_Start();
    /*Enable PWM, disable PWM IO output*/
    CtrlReg_PWMOut_Write(0x01);       
   
	/*Speed Counter Initialization*/
	Counter_Spd_Start();
	
    /*For voltage sample*/
	ADC_SAR_Seq_1_Start();
    
    /*For over current protection*/
	IDAC_Iref_Start();
    IDAC_Iref_SetValue(0x7d);
    
    LPComp_OC_Start();
	
    /*UART Init*/
    UART_BCP_Start();

	isr_pwm_Start();
	isr_pwm_StartEx(pwm_isr);
	isr_pwm_Enable();  
	
	isr_spd_Start(); 
	isr_spd_StartEx(speed_measure_isr);    
	isr_spd_Enable(); 	
    
	isr_oc_Start(); 
	isr_oc_StartEx(over_current_isr);    
	isr_oc_Disable();  
}

/*******************************************************************************
* Function Name: Init_UI_HW
********************************************************************************
* Summary: This function initializes hardware peripherals for user interface
*  
* Parameters: None  
*
* Return: None
*  
*******************************************************************************/
void Init_UI_HW()
{
	UpdateStatusInit();
}

/* [] END OF FILE */
