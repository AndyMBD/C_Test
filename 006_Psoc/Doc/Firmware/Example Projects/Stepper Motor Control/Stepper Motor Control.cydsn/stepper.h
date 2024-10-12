/*******************************************************************************
* Project Name		: Stepper Motor Control
* File Name			: stepper.h
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
#ifndef  _STEPPER_H_
#define  _STEPPER_H_

#include <cytypes.h>    

#define TABLETOP    127 

#define CCW (1u)   /* COUNTER_CLOCK_WISE */
#define CW (255u)  /* CLOCK_WISE */

typedef struct _STEPPER
{
    uint8 stageIndex;
    uint8 microStepPointer;
    uint8 microStepPace;
    uint8 sinTblPointer;
    uint8 cosTblPointer;    
} STEPPER;

/* Microstepping state initialization: **********************
* stageIndex = 0;
* microStepPointer = 0;
* microStepPace = 1;
* sinTblPointer = 0;
* cosTblPointer = 0;
*************************************************************/
#define STEPPER_DEFAULT  {0,0,1,0,0}		

extern void Stepping(void);
extern void Stopping(void);

#endif 	



/* End _STEPPER_H_ */

