/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc.h
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

/* header files for Motor Control binary library */
#include "CymcLib\CymcLib.h"
/* header files for Motor Control application layer */
#include "MAL\Cymc_MAL.h"	    
/* header files	for Motor Control HAL layer */
#include "HAL\Cymc_HAL_PWM.h"
#include "HAL\Cymc_HAL_ADC.h"


/* Assign component real name to HAL pre-defined MACRO */
#define PWMU_NAME        PWM_A			/*PWM Component Name of U Phase*/
#define PWMV_NAME        PWM_B			/*PWM Component Name of V Phase*/
#define PWMW_NAME        PWM_C			/*PWM Component Name of W Phase*/
#define PWMREG_NAME		 PWM_Ctrl_Reg	/*PWM Control Register Name*/
#define ADC_NAME		 ADC			/*SAR ADC Component name*/
#define ADCREG_NAME		 ADC_Ctrl_Reg	/*ADC Control Register Name*/	



/* [] END OF FILE */
