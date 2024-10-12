/*******************************************************************************
* Project Name		: SingleShunt Foc Motor Control
* File Name			: foc.c
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

/* [] END OF FILE */
#include <project.h>
#include "math.h"
#include "Cymc.h"

/* channel numbers for SAR */
#define ADC_CURRENT_CH          0u
#define ADC_VUBS_CH             1u
#define ADC_SPEED_POT_CH        2u
/* threshold definition for protection */
#define ALAEMCURRENT      		(int16)(5*4096*3*5/330)
#define MIN_SPEED_STEP    		_FQ(0.02)
#define SLOW_REGULATION   		_FQ(0.1)
#define MIDDLE_REGULATION 		_FQ(0.2)
#define FAST_REGULATION   		_FQ(0.4)
#define VBUS_LOW				15
#define VBUS_HIGH				30
#define SEC_COUNTER             5000

uint16 speedLoopCount = 1; 
/* phase current U, V in q15 format */
q15_t phaseCurrentU = 0, phaseCurrentV = 0;	
q15_t phaseCurrentU0 = 0, phaseCurrentV0 = 0;
/* current angle in q15 format */
q15_t curAngle = 0;
/* second counter*/
uint16 pwmCounter = 0;
uint8  secFlag = 0;
/* For Debug */
q15_t tst1,tst2,tst3,tst4;
extern q15_t kp,ki;

uint16 adcSampleIndex = 0;
CY_ISR(ISR_SARADC)
{
    if(adcSampleIndex < 2)
	{
		adcRawData[adcSampleIndex] = Cymc_HAL_ADCGetSample16(ADC_CURRENT_CH);
		if(adcSampleIndex == 0)
			Cymc_HAL_ADCEnableAllChannel();
	}
    
	if(adcSampleIndex > 0)
	{
		adcRawData[2] = Cymc_HAL_ADCGetSample16(ADC_VUBS_CH); 
		adcRawData[3] = Cymc_HAL_ADCGetSample16(ADC_SPEED_POT_CH); 
	}
	adcSampleIndex++;
}

/****************************************************************************************
*																			  			*
*   Function:    updateSpeedrefFromVR								  					*
*																			  			*
*   Description: update speed reference 									        	*
*																			  			*				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void updateSpeedrefFromVR()
{
	q15_t temp;
	q15_t refTemp;
	
    if(motor.runState != M_SPEEDLOOP)
		return;
    
	refTemp = adcRawData[3] << 3;
	refTemp = Cymc_GFL_Mul_Q15(refTemp, _FQ(1.1));
    
	temp = refTemp - motor.speedRef;    
	if(temp < 0)
		temp = -temp;
	if(temp > MIN_SPEED_STEP)
		motor.speedRef = refTemp;
    
	temp = motor.speedRef - rampUp.output;    
	if(temp < 0)
		temp = -temp;
	if(temp > SLOW_REGULATION)
	{
		rampUp.delayPrescaler = SLOW_RAMPUP_DELAY;
	}
	else if(temp > MIDDLE_REGULATION)
	{
		rampUp.delayPrescaler = MIDDLE_RAMPUP_DELAY;
	}
	else if(temp > FAST_REGULATION)
	{
		rampUp.delayPrescaler = FAST_RAMPUP_DELAY;
	}
	else
	{
		rampUp.delayPrescaler = DEFAULT_RAMPUP_DELAY;
	}

	if(motor.speedRef > SPEEDREF_MAX)
		motor.speedRef = SPEEDREF_MAX;
	if(motor.speedRef < SPEEDREF_MIN)
		motor.speedRef = SPEEDREF_MIN;
    
    return;	
}
/****************************************************************************************
*																			  			*
*   Function:    phaseCurrentOverProtect								  				*
*																			  			*
*   Description: phase current over protection 									        *
*																			  			*				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void phaseCurrentOverProtect()
{
	if(phaseCurrentU > ALAEMCURRENT  || phaseCurrentU < -ALAEMCURRENT)
	{
		errState |= ERR_OVER_CURRENT;
		Cymc_MAL_MotorStop();
	}
	if(phaseCurrentV > ALAEMCURRENT || phaseCurrentV < -ALAEMCURRENT )
	{
		Cymc_MAL_MotorStop();
		errState |= ERR_OVER_CURRENT;
	}
}
/****************************************************************************************
*																			  			*
*   Function:    VbusProtect								  							*
*																			  			*
*   Description: Under and Over Voltage protection								        *
*																			  			*				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
void VbusProtect()
{
	q15_t vBus = 0;
	vBus = ((q15_t)adcRawData[2]*33)>>11;
	if(vBus < VBUS_LOW)
	{
		Cymc_MAL_MotorStop();
		errState |= ERR_UNDER_VOLTAGE;
	}
	else if(vBus > VBUS_HIGH)
	{
		Cymc_MAL_MotorStop();
		errState |= ERR_OVER_VOLTAGE;
	}
}

/****************************************************************************************
*																			  			*
*   Function:    FOC_MainLoop_ISR								  						*
*																			  			*
*   Description: PWM main ISR, implement base FOC flow									*
*																			  			*				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
CY_ISR(FOC_MainLoop_ISR)
{
	uint16 counter = PWM_A_ReadCounter();
	if(counter > (pwmPeriod >> 1))                  /* OV event */
	{
		singleSntCal.sector = svpwmCal.Sector;
		Cymc_BCL_SetPWMOffset(&singleSntCal);
		Cymc_HAL_PWMOutputUpdate(singleSntCal.pwmCmpUp[0],singleSntCal.pwmCmpUp[1],singleSntCal.pwmCmpUp[2]);
		adcSampleIndex = 0;
		Cymc_HAL_ADCSampleEnable();
		Cymc_HAL_ADCEnableCurChannel();
        /* skip FOC processing in OV event, this ISR only change the SAR setting for single-shunt mode */
		return;
	}
	else                                            /* UN event */
	{
		Cymc_HAL_PWMOutputUpdate(singleSntCal.pwmCmpDown[0],singleSntCal.pwmCmpDown[1],singleSntCal.pwmCmpDown[2]);
		Cymc_HAL_ADCSampleDisable();
		adcSampleIndex = 0;
		Cymc_BCL_GetSampleTime(&singleSntCal);
		ADC_Timer_WritePeriod(singleSntCal.sample1Time);
		ADC_Timer_WriteCompare(singleSntCal.sample2Time);
	}
	
	/* Speed Ramp Up*/
    rampUp.RefSpeed = motor.speedRef;	
    Cymc_BCL_RampUp(&rampUp);
	if(rampUp.output < _FQ(0.2))
		motor.vqRef = (rampUp.output>>1) + _FQ(0.06);
		
	pwmCounter++;
	if(pwmCounter >= SEC_COUNTER)
	{
		pwmCounter = 0;
		secFlag = 1;
	}
	/* Angle Generator For Open Loop*/
	angleGen.speed = rampUp.output;
	Cymc_BCL_AngleGenerator(&angleGen);

	/* Read ADC and Get phase current*/
	Cymc_BCL_ReconstructADC(&singleSntCal);
	singleSntCal.lastSector = svpwmCal.Sector;
	if(motor.runState == M_STOP)
		Cymc_BCL_GetADCOffset(adcRawData[0], adcRawData[1]);
        
    phaseCurrentU = singleSntCal.adcValue[0];
    phaseCurrentV = singleSntCal.adcValue[1];
    phaseCurrentU = (phaseCurrentU - adcOffsetU);
	phaseCurrentV = (phaseCurrentV - adcOffsetV);	
    
    if(motor.runState != M_STOP)
    {
        VbusProtect();
    	if(motor.runState != M_OPENLOOP)
    		phaseCurrentOverProtect();	
    }

	phaseCurrentU0 = phaseCurrentU<<3;
	phaseCurrentV0 = phaseCurrentV<<3;

	/* Clark Transformation uvw->αβ		*/
	clarkTrans.inU = (phaseCurrentU0<<2);/* Phase U current.  */
	clarkTrans.inV = (phaseCurrentV0<<2);/* Phase V current.  */	
	Cymc_BCL_Clark(&clarkTrans); 

	/*Switch to Speed Loop*/
	if(motor.runState == M_OPENLOOP)
    {
		pidId.iOut = 0;
		pidIq.iOut = motor.vqRef>>2;
		pidSpeed.iOut = motor.vqRef>>2;
        if(rampUp.output >= motor.speedRef - _FQ(0.0000610))
        {
            motor.runState = M_SPEEDLOOP;//M_SPEEDLOOP;
			if(motor.runState != M_CURRENTLOOP || motor.runState != M_CURRENTLOOP)
			{
				pidId.iOut = 0;
				pidIq.iOut = motor.vqRef;
			}
        }
    }
	/* Position Calculation*/
	smoEst.Vbus = _FQ(1.0);
	smoEst.speedRef = rampUp.output;
	smoEst.volAlpha = invParkTrans.outAlpha;
	smoEst.volBeta = invClarkTrans.InBeta;
	smoEst.curAlpha = clarkTrans.outAlpha;
  	smoEst.curBeta  = clarkTrans.outBeta;
	Cymc_ACL_SMOCal(&smoEst);
	
    /* Speed Calculation*/
	if(motor.runState != M_STOP)
	{
	    speedEst.curTheta = curAngle;
		Cymc_BCL_SpeedCal(&speedEst);
		motor.speedRPM = speedEst.estimatedSpeedRpm;
        /* speed value cannot be negative */
        if(motor.speedRPM < 0)
            motor.speedRPM = 0;
        if((uint16)motor.speedRPM > SPEED_RPM_MAX)
            motor.speedRPM = SPEED_RPM_MAX;
	}
	else
	{
		speedEst.estimatedSpeed = 0;
		motor.speedRPM = 0;
	}

	/*angle given*/
    if(motor.runState == M_SPEEDLOOP)
    {
		curAngle = smoEst.Theta;
    }
    else
    {
        curAngle = angleGen.angle;
		pidSpeed.iOut = 0;
    }
	
	/* Park Transformation αβ->dq		*/
	parkTrans.inAlpha = clarkTrans.outAlpha;
    parkTrans.inBeta  = clarkTrans.outBeta;
	Cymc_GFL_SinCosCal_Q15(curAngle, &parkTrans.inSin,  &parkTrans.inCos);
    Cymc_BCL_Park(&parkTrans);
    
	/* Speed PID Controller		*/
    if (speedLoopCount == speedLoopPrescaler)
    {
        pidSpeed.ref = rampUp.output; /* motor.speedRef; */
        pidSpeed.fbk = speedEst.estimatedSpeed;
		Cymc_GFL_PICal(&pidSpeed);
        speedLoopCount    = 0;
		updateSpeedrefFromVR();
    }
	speedLoopCount++; 
    
	/* Iq PID Controller		*/
    if(motor.runState == M_SPEEDLOOP)
    {	
		pidIq.ref = pidSpeed.out;
	}
    else
    {
		pidIq.ref = motor.vqRef;
	}
    pidIq.fbk = parkTrans.outQ;
	Cymc_GFL_PICal(&pidIq);
	
	/* Id PID Controller		*/
    pidId.ref = motor.vdRef; 
    pidId.fbk = parkTrans.outD;
    Cymc_GFL_PICal(&pidId);
	
	/* Inverse Park Transformation dq->αβ	*/
    if(motor.runState == M_OPENLOOP || motor.runState == M_STOP)
    {
        invParkTrans.inD = motor.vdRef;
        invParkTrans.inQ = motor.vqRef;
    }
    else 
    {
        invParkTrans.inD = pidId.out;
        invParkTrans.inQ = pidIq.out;
    }
    invParkTrans.inSin   = parkTrans.inSin;
    invParkTrans.inCos = parkTrans.inCos;
    Cymc_BCL_InvPark(&invParkTrans);
    
	/* Inverse Clark Transformation αβ->uvw		*/
    invClarkTrans.InAlpha = invParkTrans.outAlpha;
	invClarkTrans.InBeta = invParkTrans.outBeta;
	Cymc_BCL_InvClark(&invClarkTrans);
	
	/* SVPWM Calculation			*/
	svpwmCal.Uu = invClarkTrans.outU;
	svpwmCal.Uv = invClarkTrans.outV;
	svpwmCal.Uw = invClarkTrans.outW;
    Cymc_BCL_SVPWM(&svpwmCal);
	
 	/* Update PWM Duty		*/
	pwmCmp[0] = pwmPeriod - svpwmCal.CmpU;
	pwmCmp[1] = pwmPeriod - svpwmCal.CmpV;
	pwmCmp[2] = pwmPeriod - svpwmCal.CmpW;
}

/****************************************************************************************
*																			  			*
*   Function:    Ibus_Over_ISR								  							*
*																			  			*
*   Description: comparator ISR for bus current over protection							*
*																			  			*				
*   Parameters:   None																	*
*				  							  			                                *
*   Returns:     None														  			*		
*																			  			*		
****************************************************************************************/
CY_ISR(Ibus_Over_ISR)
{
	LPComp_IbusPt_ClearInterrupt(LPComp_IbusPt_INTR);
//	if(LPComp_IbusPt_GetCompare())
//	{            
//		errState |= ERR_OVER_CURRENT;
//		Cymc_MAL_MotorStop();
//		LPComp_IbusPt_ClearInterrupt(LPComp_IbusPt_INTR);
//	}
	LPComp_IbusPt_ClearInterrupt(LPComp_IbusPt_INTR);
}
/* [] END OF FILE */
