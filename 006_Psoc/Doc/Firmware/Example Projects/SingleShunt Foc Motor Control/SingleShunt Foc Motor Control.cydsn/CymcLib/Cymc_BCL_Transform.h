/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_BCL_Transform.h
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
#ifndef __CYMC_BCL_TRANSFORM_H__
#define __CYMC_BCL_TRANSFORM_H__
#include "Cymc_GFL_Math.h"
	
/* struct definition for clark transformation*/
	typedef struct 
	{  
		q15_t  inU;  		/* phase-U variable    	*/
		q15_t  inV;			/* phase-V variable    	*/
		q15_t  outAlpha;	/* Output: α-axis variable    	*/
		q15_t  outBeta;		/* Output: β-axis variable    	*/
	} ClarkTrans;
/* struct definition for inverse clark transformation*/	
	typedef struct 
	{  
		q15_t  outU;  		/* Output: phase-U variable	  	*/
		q15_t  outV;		/* Output: phase-V variable	  	*/
		q15_t  outW;		/* Output: phase-W variable	  	*/
		q15_t  InAlpha;	    /* α-axis  variable   	*/
		q15_t  InBeta;		/* β-axis  variable	  	*/
	} InvClarkTrans;
/* struct definition for park transformation*/		
	typedef struct 
	{  
		q15_t  inAlpha;  	/* α-axis  variable 	*/
		q15_t  inBeta;	 	/* β-axis  variable 	*/
		q15_t  outD;		/* Output: d-axis  variable 	*/
		q15_t  outQ;		/* Output: q-axis  variable		*/
		q15_t  inSin;		/* angle sin  value 	*/
		q15_t  inCos; 		/* angle cos  value 	*/	 
	} ParkTrans;
/* struct definition for inverse park transformation*/
	typedef struct 
	{  
		q15_t  outAlpha;  	/* α-axis  variable     */
		q15_t  outBeta;	 	/* β-axis  variable 	*/
		q15_t  inD;		    /* Output: d-axis  variable 	*/
		q15_t  inQ;		    /* Output: q-axis  variable		*/
		q15_t  inSin;		/* angle sin  value 	*/
		q15_t  inCos; 	 	/* angle cos  value 	*/	
	} InvParkTrans;

void Cymc_BCL_Clark(ClarkTrans *clark);
void Cymc_BCL_Park(ParkTrans *park);
void Cymc_BCL_InvPark(InvParkTrans *invPark);
void Cymc_BCL_InvClark(InvClarkTrans *invClark);

#endif
/* [] END OF FILE */
