/*******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: stepper.c
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

/* 128 sinusoid table */
static unsigned char SinTable[] = {
 127,127,127,127,127,127,127,127,126,126,
 126,126,126,125,125,125,125,124,124,124,
 123,123,122,122,121,121,120,120,119,119,
 118,118,117,117,116,115,115,114,113,113,
 112,111,110,109,109,108,107,106,105,104,
 103,103,102,101,100,99,98,97,96,95,
 94,93,91,90,89,88,87,86,85,83,
 82,81,80,79,77,76,75,74,72,71,
 70,68,67,66,64,63,62,60,59,58,
 56,55,54,52,50,49,48,46,45,43,
 42,40,39,37,36,34,33,31,30,28,
 27,25,23,22,20,19,17,16,14,13,
 11,9,8,6,5,3,2,0
 };

STEPPER Sm = STEPPER_DEFAULT;
uint8  currRef_A = 0;
uint8  currRef_B = 0;
extern UI_DATA UI_Data;


/*******************************************************************************
* Function Name: Stepping
********************************************************************************
*
* Summary:
* Step function runs motor one micro step. This function is called in Timer ISR.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void Stepping(void)
{

	/* When 1/4 Sinusoidal wave end, change the stage of run-on phase*/
   if(Sm.microStepPointer == 0)		 
   {    
        Sm.microStepPace = UI_Data.microStepPace;     	

        Sm.stageIndex += UI_Data.dir;
        Sm.stageIndex &= 0x03; 
	
        Control_Reg_1_Write(Sm.stageIndex);		
    }

	/* Get the Sinusoidal Current Reference value for phase A and B*/	 
    if(UI_Data.dir == CCW)									
    {
        if ((Sm.stageIndex & 0x01) == 0)		/* Stage 0 & 2: Current Down */				 
            Sm.sinTblPointer = Sm.microStepPointer;
        else  									/* Stage 1 & 3: Current Up */
            Sm.sinTblPointer = TABLETOP - Sm.microStepPointer ; 
    }
    else
    {
        if ((Sm.stageIndex & 0x01) == 1)		
            Sm.sinTblPointer = Sm.microStepPointer;
        else  						
            Sm.sinTblPointer = TABLETOP - Sm.microStepPointer;
    }
            
    Sm.cosTblPointer = TABLETOP - Sm.sinTblPointer;

    /* Set Sinusoidal current reference for phase A and B */
    currRef_A = SinTable[Sm.sinTblPointer];    
    currRef_B = SinTable[Sm.cosTblPointer];
	
	IDAC_A_SetValue(currRef_A);
	IDAC_B_SetValue(currRef_B);

    
    Sm.microStepPointer += Sm.microStepPace;
    Sm.microStepPointer &= 0x7F; 
	 
}

/*******************************************************************************
* Function Name: Stopping
********************************************************************************
*
* Summary:
* Stop function stops motor and reduces its current for power saving.
*
* Parameters: None
*
* Return: None
*
*******************************************************************************/
void Stopping(void)
{   
    /* Reset index to zero*/
    Sm.stageIndex = 0;

    currRef_A = SinTable[Sm.sinTblPointer];
    currRef_A >>=1;		/* Reduce to 1/2 of normal value */
	IDAC_A_SetValue(currRef_A);	
    
    currRef_B = SinTable[Sm.cosTblPointer];
    currRef_B >>=1;		/* Reduce to 1/2 of normal value */
	IDAC_B_SetValue(currRef_B);	
}
/* End of File */
