/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_MAL.h
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
#ifndef __CYMC_MAL_H__
#define __CYMC_MAL_H__
#include <cytypes.h>	
#include "CymcLib\CymcLib.h"

#define PI              3.14159265358979

#define M_STOP	  		0
#define M_OPENLOOP   	1
#define M_CURRENTLOOP   2
#define M_SPEEDLOOP     3	
	
#define SPEED_MIN_STEP  _FQ(0.0000610)	
	
#define M_RS            0.8
#define M_LS            0.0012
#define M_POLES         8	
#define M_TS            0.000125
#define M_RATED_SPEED   4000u    
#define M_BASEFREQUENCY 266.67

#define ERR_OVER_CURRENT     (1<<0)
#define ERR_OVER_VOLTAGE     (1<<1)
#define ERR_UNDER_VOLTAGE    (1<<2)	
#define ERR_START_FAILED     (1<<3)
#define ERR_LOSE_STEP        (1<<4)	
	

#define PID_ID_KP       _FQ(1.0)
#define PID_ID_KR       _FQ(1.0)
#define PID_ID_KI       _FQ(0.2)
#define PID_ID_OMAX     _FQ(0.7)
#define PID_ID_OMIN     _FQ(-0.7)	
	
#define PID_IQ_KP       _FQ(1.0)
#define PID_IQ_KR       _FQ(1.0)
#define PID_IQ_KI       _FQ(0.2)
#define PID_IQ_OMAX     _FQ(0.95)
#define PID_IQ_OMIN     _FQ(-0.95)
	
#define PID_SPEED_KP    _FQ(0.8333)
#define PID_SPEED_KR    _FQ(1.0)
#define PID_SPEED_KI    _FQ(0.0033264)
#define PID_SPEED_OMAX  _FQ(0.55)
#define PID_SPEED_OMIN  _FQ(-0.55)	

#define DEFAULT_SPEEDLOOPSCALER	10
#define DEFAULT_RAMPUP_DELAY    50
#define SLOW_RAMPUP_DELAY       50	
#define MIDDLE_RAMPUP_DELAY     50
#define FAST_RAMPUP_DELAY       50	

#define SPEED_RPM_MAX   M_RATED_SPEED                       /* rated speed */
#define SPEED_RPM_MIN   500u                                /* 500 RPM */  
#define SPEEDREF_MAX    _FQ((float)SPEED_RPM_MAX/M_RATED_SPEED)
#define SPEEDREF_MIN    _FQ((float)SPEED_RPM_MIN/M_RATED_SPEED)	
 
/* struct definition for motor controller*/
typedef struct 
{
   float Rs;  	         /* motor Rs 			*/ 
   float Ls;   		     /* motor Ls			*/
   uint8 Poles;          /* motor Poles			*/
   float baseFrequency;  /* base frequency		*/
   float Ts;  	         /* PWM Period			*/
   int16 ratedSpeedRPM; /* rated Speed in RPM Format */
   int16 speedRPM;      /* Speed in RPM Format */
   uint8 runState;	     /* Motor run state  	*/
   q15_t speedRef; 		 /* speed reference		*/
   q15_t vdRef; 		 /* vd reference		*/
   q15_t vqRef; 		 /* vq reference		*/
} MotorController; 
	
extern MotorController motor;	
extern RampUp         rampUp;
extern AngleGenerator angleGen;
extern ClarkTrans     clarkTrans;
extern ParkTrans      parkTrans;
extern InvParkTrans   invParkTrans;
extern PIController   pidSpeed;
extern PIController   pidId;
extern PIController   pidIq;
extern SpeedEstimator speedEst;
extern InvClarkTrans  invClarkTrans;
extern SVPWM          svpwmCal;
extern SMO            smoEst;
extern SingleShuntCalculator singleSntCal;	
extern 	uint16 speedLoopPrescaler;
extern uint16  errState;
	
extern void Cymc_MAL_MotorInit(void);	
extern void Cymc_MAL_MotorStart(void);
extern void Cymc_MAL_MotorStop(void);


#endif	
/* [] END OF FILE */
