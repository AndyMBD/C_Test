/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_GFL_PID.h
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
#ifndef __CYMC_GFL_PID_H__
#define __CYMC_GFL_PID_H__

#include "Cymc_GFL_Math.h"
    
typedef struct {  
				  q15_t  ref;   			/* reference set-point	*/
				  q15_t  fbk;   			/* feedback				*/
				  q15_t  out;   			/* Output: controller output 	*/
				  q15_t  kr;				/* reference set-point weighting	*/ 
				  q15_t  kp;				/* proportional loop gain		*/
				  q15_t  ki;			    /* integral gain					*/
				  q15_t  outMax;			/* upper output limit		*/
				  q15_t  outMin;			/* lower output limit		*/
				  q15_t  pOut;				/* proportional term					*/
				  q15_t  iOut;				/* integral term						*/
				  q15_t  outPre;			/* pre-saturated controller output	*/
				} PIController;


void Cymc_GFL_PICal(PIController *v);
#endif
/* [] END OF FILE */
