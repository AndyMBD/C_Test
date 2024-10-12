/*******************************************************************************
* Project Name		: Sensored BLDC Motor Control
* File Name			: getvalue.h
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
#ifndef _GETVALUE_H_
#define _GETVALUE_H_
    
#include <cytypes.h>

#define  MAX_CURR_LOW         50        /* 1.0A */
#define  MAX_CURR_MEDIUM      75        /* 1.5A */
#define  MAX_CURR_HIGH        100       /* 2.08A */

#define LVTHRESHOLD  40                 /* 16V */
#define HVTHRESHOLD  90                /* 36V */  
	
#define LVCNTTHRES  2560                 
#define HVCNTTHRES  2560                

/* ADC channel number for speed reference potentiometer */     
#define SPEED_REF_CHAN                      ((uint8)(1))     
/* ADC result range */    
#define ADC_RANGE                           ((uint16)(4096))      
/* define the maximum resistor value for potentiometer */    
#define SPEED_POT_VALUE                     (uint16)(10000)    
/* define the accepted maximum/minimum value for the sample result of speed reference potentiometer,
   minus 100 to reduce the offset at the boundary*/    
#define MAX_SPEED_REF_ADC_VALUE             (uint16)((((SPEED_POT_VALUE + 330) * ADC_RANGE)/(SPEED_POT_VALUE + 330 + 330)) - 30)    
#define MIN_SPEED_REF_ADC_VALUE             (uint16)((((330) * ADC_RANGE)/(SPEED_POT_VALUE + 330 + 330)) + 30)        

void VoltageCheck(void);
uint16 ReadRpmRef(void);

#endif  /* _GETVALUE_H_ */

