/*******************************************************************************
* Project Name		: Sensorless BLDC Motor Control
* File Name			: Startup.c
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
#include "Startup.h"
#include "MotorRun.h"
#include "BLDCController.h"

/******************************************************************************
 * Global variables definition
 * ----------------------------------------------------------------------------
 * These variables should be populated to other modules. Header file contains 
 * the extern statement for these variables.
 ******************************************************************************/ 

/******************************************************************************
 * Local Macro definition
 * ----------------------------------------------------------------------------
 * These Macros are only used in this module. These Macros should not be populated
 * to other modules.
 ******************************************************************************/

/******************************************************************************
 * Local variables definition
 * ----------------------------------------------------------------------------
 * These variables are only used in this module. These variables should not be 
 * populated to other modules.
 ******************************************************************************/
/* timeout flag for zero crossing checking in freerun stage */
static uint8 timeoutFlag = TRUE;

typedef enum _Startup_Stage_T 
{
    START_STAGE,
    DEC_VOLT_STAGE,
    FREE_RUN_STAGE,
    ACCELERATION_STAGE,
    FAIL_STAGE
}Startup_Stage_T;

/*******************************************************************************
* Function Name: Preposition
********************************************************************************
* Summary:
*   align rotor to desired position with pre-defined duty and time 
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void Preposition(void)
{
    /* counter for pre-position stage */
    static uint32  prepositionTicker = 0;
    
    /* Set duty cycle of preposition */
    dutyCycle = BLDC_Config.prepositionDuty;
    PWM_Drive_WriteCompare1(dutyCycle);  
    
    SectorCtrl_Write(1);
    /* Delay 100mS */
    if(prepositionTicker >= BLDC_Config.prepositionTime)
    {  
        /* Set duty cycle of Freerun */
        dutyCycle = BLDC_Config.startDuty;
        /* Change status */
        runState  = FREERUNNING;    
        /* Initialize parameters for freerun */        
        timeoutFlag = TRUE;
        /* reset preposition ticker */        
        prepositionTicker = 0;
    }
    else
    {
        prepositionTicker++;
    }
}

/*******************************************************************************
* Function Name: FreeRun
********************************************************************************
* Summary:
*   Detect initial zero crossing point, sampling the BEMF signal based on the output
*   of comparator have impact pulse.
*
* Parameters:  
*  void:  
*
* Return: 
*  void
*
*******************************************************************************/
void FreeRun(void)
{      
    /* pointer of startup time table */
    static uint16 freerunCounter = 0;
    /* checking timer in free run stage */
    static uint32  checkPeriod = 1;
    /* flag to enable zero-crossing check */
    static uint8 checkEnable = FALSE;
    /* detected zero crossing count */
    static uint8 zeroCrossingCount = 0;
    /* state for startup procedure */
    static Startup_Stage_T startupStage = START_STAGE;
    /* counter to store how many commutations happen in specific period */
    static uint8 resetCount = 0;    
    static uint16 commutateCount;
    
    uint16 curCount = 0;     
    uint16 dutyLocal = dutyCycle;
    
    if(timeoutFlag == TRUE)
	 {   
        /* reset timeoutFlag after a detect timeout happens */
	    timeoutFlag = FALSE;         
        /* execute one commutation in open loop */
	    Commutating();
        /* store the commutation timestamp */
        commutateCount = Counter_Spd_ReadCounter();
        		       
        /* reset counter for zero-crossing after desired commutation happens */
		if(resetCount >= ZC_COUNT_RESET_THRESHOLD)
		{
			resetCount = 0;
			zeroCrossingCount = 0;
		}
        resetCount++;  
        
        /* Have enough ZC count and ready to switch to normal run stage */
        if((zeroCrossingCount >= ZC_CHECKING_STABLE_THRESHOLD) && (resetCount <= 5))
		{           
            /* reset all internal flags for next execution of startup freerun */
            startupStage = START_STAGE;
            resetCount = 0;
			zeroCrossingCount = 0;			
            freerunCounter = 0;
            checkEnable = FALSE;
            timeoutFlag = TRUE;
            BLDC_Config.accStageWait = ACC_STAGE_EXEC_COUNT;
            
            /* switch main state machine to normal run state */
            runState = NORMALRUN;
            /* initialize the first commutate period based on the last commutation in freerun stage */
            BLDC_Control.zeroCrossPeriod = checkPeriod;           
            /* set flag to indicated that motor runs in normal run stage */
            BLDC_Control.inNormalRun = TRUE; 
                                  
            /* get timestamp for current commutation */
            BLDC_Control.commutateStamp = commutateCount;
            /* down counter, avoid an unexpected interrupt generation */
            Counter_Spd_WriteCompare(Counter_Spd_ReadCounter() + 10);          
           
            return;
		}
        
        /* state machine for startup */
        switch(startupStage)
		{
		case START_STAGE:
            /* --------------------------------------------------------------------------------------
             * In this state, the voltage on motor winding is fixed, the rotation speed is also fixed.
             * This state intends to make motor start to run with a big enough start voltage. 
             * Do not detect zero crossing in this stage 
             * --------------------------------------------------------------------------------------*/
            checkEnable = FALSE;
            dutyLocal = BLDC_Config.startDuty;
            checkPeriod = BLDC_Config.startCheckPeriod;
                        
			if(freerunCounter >= BLDC_Config.startStageWait)
			{
                /* switch stage for startup process */
				startupStage = DEC_VOLT_STAGE;
                /* reset internal counter */
				freerunCounter = 0;
			}
			break;
            
		case DEC_VOLT_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is decreased, the rotation speed is still 
             * fixed. This state intends to decrease the voltage to prevent motor winding from burn.
             * Do not detect zero crossing in this stage 
             * --------------------------------------------------------------------------------------*/       
			checkEnable = TRUE;
            /* only decrease duty cycle when reaching the end of interval */
            if(freerunCounter >= BLDC_Config.decStageInterval)
			{
				dutyLocal--;
				freerunCounter = 0;
                /* increase checking speed */
                checkPeriod -= 3;
			}
            
			if(dutyLocal <= BLDC_Config.freerunDuty)
            {
				startupStage = FREE_RUN_STAGE;
                freerunCounter = 0;
            }
            
			break;
            
		case FREE_RUN_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is set with free run voltage, which may be 
             * lower than start voltage, because the start voltage needs to overcome the big inertia
             * at heavy load status. The rotation speed is still fixed. 
             * Start to detect zero crossing in this stage 
             * --------------------------------------------------------------------------------------*/            
			checkEnable = TRUE;
            dutyLocal = BLDC_Config.freerunDuty;
            checkPeriod = BLDC_Config.startCheckPeriod;
            
			if(freerunCounter >= BLDC_Config.freerunStageWait)
			{
				startupStage = ACCELERATION_STAGE;
				freerunCounter = 0;
			}
			break;
            
		case ACCELERATION_STAGE:
            /* --------------------------------------------------------------------------------------
             * in this state, the voltage on motor winding is increased for higher speed. The rotation
             * speed is also increased.
             * --------------------------------------------------------------------------------------*/             
			checkEnable = TRUE;
            if(freerunCounter >= BLDC_Config.accStageInterval)
			{
				freerunCounter = 0;
				dutyLocal += BLDC_Config.accDutyStep;
				checkPeriod -= BLDC_Config.accTimeStep;
				BLDC_Config.accStageWait--;
			}
			
			if(BLDC_Config.accStageWait == 0)
			{
				startupStage = FAIL_STAGE;
				freerunCounter = 0;
			}
			break;
	
		case FAIL_STAGE:
        default:
            runState = FREERUNFAILED;
            BLDC_Control.errorCode = FREERUN_ERROR;
			break;
		}
        
        dutyCycle = dutyLocal;
        freerunCounter++;
	}
	   
	/* get current count from down counter */
    curCount = Counter_Spd_ReadCounter();
    /* since it is down counter, the commutate count should be larger than current count */
	if((uint16)(commutateCount - curCount) < checkPeriod)
	{
	    if(checkEnable)
		{
            /* check zero-crossing event, if yes, zeroCrossingIsValid will be true */
		    if(CheckZeroCrossing() == TRUE)
			{
                /* increase zero-crossing counter */
                zeroCrossingCount++;
                /* initialize the last zero-crossing time stamp */
                BLDC_Control.zeroCrossStamp = curCount;
                /* Exit from zero crossing detection */
			    checkEnable = FALSE;                  
			}
		}
	}	
	else
	{
        /* If timeout, execute next forcing commutation */
	    timeoutFlag =TRUE;	
	} 
}

/* [] END OF FILE */
