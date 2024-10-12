/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_HAL_PWM.h
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
#ifndef _CYMC_HAL_PWM_H_
#define _CYMC_HAL_PWM_H_

#include <cytypes.h>
    
/*PWM Period*/
extern uint16 pwmPeriod;
/*PWM Compare Values*/	
extern uint16 pwmCmp[3];

void Cymc_HAL_PWMStart(void);
void Cymc_HAL_PWMStop(void);
void Cymc_HAL_PWMOutputEnable(void);
void Cymc_HAL_PWMOutputDisable(void);
#if SIGNLE_SHUNT
void Cymc_HAL_PWMOutputUpdate(uint16 PWMU, uint16 PWMV, uint16 PWMW);
#else
inline void Cymc_HAL_PWMOutputUpdate();
#endif


#endif  // _PSOC_FOC_PWM_H_
//[] END OF FILE
