/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_BCL_Control.h
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
#ifndef __CYMC_BCL_CONTROL_H__
#define __CYMC_BCL_CONTROL_H__
#include <cytypes.h>	
#include "Cymc_GFL_Math.h"
	
/* struct definition for speed estimator*/
    typedef struct 
	{
       q15_t curTheta;  	        /* Current Electrical angle			*/ 
       q15_t oldTheta;   		    /* Old Electrical angle				*/
       q15_t estimatedSpeed;        /* Estimated speed in per-unit		*/
       q15_t baseRpm;     		    /* Base speed in rpm				*/
       q15_t intervalConst;       	/* Constant for differentiator  	*/
       q15_t filterConst;     		/* Constant for low-pass filter		*/
       q15_t estimatedSpeedRpm;     /* Estimated speed in rpm  		    */
    } SpeedEstimator;  	
/* struct definition for position angle generator*/	
	typedef struct 
	{ 
		q15_t  speed; 				/* given speed  	      			*/
		q15_t  stepAngle;				
	 	q15_t  angle;				/* Step angle 				  		*/	  			 
	} AngleGenerator;
/* struct definition for speed ramp up*/	
	typedef struct 
	{ 
		q15_t    RefSpeed; 		 	/* reference speed 					*/
		q15_t 	 delayPrescaler; 	/* delay  Prescaler     			*/
		q15_t    stepSpeed;
		q15_t    output;		 	/* output speed						*/		 
  	} RampUp;

/* struct definition for single-shunt*/	
	typedef struct 
	{ 
		uint16 pwmCmpUp[3];	
		uint16 pwmCmpDown[3];
		uint16 *orgPwmCmp;
		uint16 pwmPeriod;
		uint16 sector;
		uint16 lastSector;
		uint16 *adcRawData;
		uint16 adcValue[3];
		uint16 adcOffset;
		uint16 Tmin;
		uint16 sample1Time;
		uint16 sample2Time;
  	} SingleShuntCalculator;
	
/* ADC offset for U, V phase current*/	
extern q15_t adcOffsetU, adcOffsetV;

void Cymc_BCL_SpeedCal(SpeedEstimator *speedEst);
void Cymc_BCL_AngleGenerator(AngleGenerator *angleGen);
void Cymc_BCL_RampUp(RampUp *rampUp);
void Cymc_BCL_GetADCOffset(uint16 adcU, uint16 adcV);

void Cymc_BCL_SetPWMOffset(SingleShuntCalculator *ssc);
void Cymc_BCL_GetSampleTime(SingleShuntCalculator *ssc);
void Cymc_BCL_ReconstructADC(SingleShuntCalculator *ssc);

#endif	
/* [] END OF FILE */
