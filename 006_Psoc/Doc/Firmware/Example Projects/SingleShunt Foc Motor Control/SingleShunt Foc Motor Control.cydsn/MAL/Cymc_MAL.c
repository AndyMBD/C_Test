/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: Cymc_MAL.c
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
#include "MAL\Cymc_MAL.h"
#include "HAL\Cymc_HAL_PWM.h"
#include "HAL\Cymc_HAL_ADC.h"


uint16 speedLoopPrescaler = DEFAULT_SPEEDLOOPSCALER;      /* Speed loop prescaler */
uint16  errState = 0;

MotorController motor;   /* motor control struct */


RampUp         rampUp;       /* motor control struct */
AngleGenerator angleGen;     /* angle generator struct */
ClarkTrans     clarkTrans;   /* clark transformation struct */
ParkTrans      parkTrans;    /* park transformation struct */
InvParkTrans   invParkTrans; /* inverse park transformation struct */
PIController   pidSpeed;     /* speed PID controller */
PIController   pidId;        /* Id PID controller */
PIController   pidIq;        /* Iq PID controller */
SpeedEstimator speedEst;     /* speed estimator struct */
InvClarkTrans  invClarkTrans;/* inverse clark transformation struct */
SVPWM          svpwmCal;     /* svpwm struct */
SMO            smoEst;       /* slider mode observer struct */
SingleShuntCalculator singleSntCal; /**/

/****************************************************************************************
*																			  			*
*   Function:    Cymc_MAL_MotorInit										  			    *
*																			  			*
*   Description: Init motor lib struct parameters and motor parameters		            *
*																			  			*				
*   Parameters:   None 																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void Cymc_MAL_MotorInit()
{
	motor.Rs = M_RS;	           /* ohm */
	motor.Ls = M_LS;              /* H */
	motor.Poles = M_POLES;
	motor.Ts = M_TS;              /* S */
	motor.baseFrequency = M_BASEFREQUENCY; /* Hz */
	motor.ratedSpeedRPM = M_RATED_SPEED;   /* RPM */
	
	/* Initialize the angle generator parameters */
	angleGen.stepAngle = _FQ(motor.baseFrequency*motor.Ts);
	
	/* Initialize the SMO parameters */
    Cymc_ACL_SMOParamtersInit(motor.Rs, motor.Ls, motor.Ts, motor.baseFrequency, &smoEst);
	
    /* Initialize the Id PID parameters */ 
	pidId.kp   	= PID_ID_KP;
    pidId.kr   	= PID_ID_KR;
	pidId.ki   	= PID_ID_KI;
    pidId.outMax = PID_ID_OMAX;
    pidId.outMin = PID_ID_OMIN;
 
	/* Initialize the Iq PID parameters */ 
	pidIq.kp   	= PID_IQ_KP;
    pidIq.kr   	= PID_IQ_KR;
	pidIq.ki   	= PID_IQ_KI;
    pidIq.outMax = PID_IQ_OMAX;
    pidIq.outMin = PID_IQ_OMIN;
    
	/* Initialize the Speed PID parameters */
    pidSpeed.kp   = PID_SPEED_KP;
    pidSpeed.kr   = PID_SPEED_KR;
	pidSpeed.ki   = PID_SPEED_KI;
    pidSpeed.outMax = PID_SPEED_OMAX;
    pidSpeed.outMin = PID_SPEED_OMIN;
	
	/* Initialize the Speed Estimator parameters */    
    speedEst.intervalConst = _FQ(1/(motor.baseFrequency*motor.Ts));
    speedEst.filterConst = _FQ(1/(1 + motor.Ts*2*PI*5));  
    speedEst.baseRpm = 120*(motor.baseFrequency/motor.Poles);
	
	/* Initialize the SVPWM module parameters */
	svpwmCal.Period = pwmPeriod;
	
	/**/
	singleSntCal.orgPwmCmp = pwmCmp;
	singleSntCal.adcRawData = adcRawData;
	singleSntCal.pwmPeriod = pwmPeriod;
	singleSntCal.Tmin = 240;
	singleSntCal.adcOffset = 2058;
	
	adcOffsetU = 2058;//2040;
	adcOffsetV = 2058;//2040;
	
	errState = 0;
}
/****************************************************************************************
*																			  			*
*   Function:    Cymc_MAL_MotorStart										  			*
*																			  			*
*   Description: Init motor startup parameters and enable PWM output  		            *
*																			  			*				
*   Parameters:   None 																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void Cymc_MAL_MotorStart()
{
	motor.speedRef  = _FQ(0.16);
    motor.vqRef = _FQ(0.13);			 
    motor.vdRef = _FQ(0);
	rampUp.output = 0;
	rampUp.stepSpeed = 5;
	rampUp.delayPrescaler = 80;
	motor.vqRef = (rampUp.output>>1) + _FQ(0.06);
	ADC_Timer_Start();
    Cymc_MAL_MotorInit();
	motor.runState = M_OPENLOOP;
	Cymc_HAL_PWMOutputEnable();
}
/****************************************************************************************
*																			  			*
*   Function:    Cymc_MAL_MotorStop										  				*
*																			  			*
*   Description: Disable PWM output and stop motor 		            					*
*																			  			*				
*   Parameters:   None 																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void Cymc_MAL_MotorStop()
{
	motor.runState = M_STOP;
	Cymc_HAL_PWMOutputDisable();
}

/* [] END OF FILE */
