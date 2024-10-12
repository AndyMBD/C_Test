/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: motor_ctrl.c
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


/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include <stdlib.h>
#include <project.h>

#include "h01_module\math.h" 

#include "h02_app\limitation.h"

#include "h02_app\motor_ctrl.h"
#include "h02_app\protect.h"

#include "h02_app\observer_pll.h"

#include "h02_app\motor_startup.h"
#include "h03_user\hardware_config.h"
#include "define.h"
#include "h01_module\svpwm.h"
#include "h03_user\chip_init.h"
#include "h03_user\interface.h"
/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

// sample
stc_uvw_t               MotorCtrl_stcIuvwSensed;

// coordinate transformation
stc_ab_t                MotorCtrl_stcIabSensed;
stc_dq_t                MotorCtrl_stcIdqSensed;
stc_dq_t                MotorCtrl_stcVdqRef;
stc_dq_t                MotorCtrl_stcIdqRef;
stc_ab_t                MotorCtrl_stcVabRef;
stc_ab_t                MotorCtrl_stcVabReal;
stc_uvw_t               MotorCtrl_stcVuvwRef;

stc_dq_t                MotorCtrl_stcEdq;
stc_ab_t                MotorCtrl_stcEab;

//stc_Ipd_t               stcIpd;

// LPF

stc_one_order_lpf_t     MotorCtrl_stc1stLpfWm = { Q12(0.1), Q20(0.0) };//0.25

// PID
stc_pid_cfg_t               MotorCtrl_stcIqPidReg;
stc_pid_cfg_t               MotorCtrl_stcIdPidReg;
stc_pid_cfg_t               MotorCtrl_stcWmPidReg;

// PWM
stc_svm_gen_t           MotorCtrl_stcSvmGen;
stc_svm_calc_t          MotorCtrl_stcSvmCalc;

//motor ctrl
stc_motor_run_t         MotorCtrl_stcRunPar;
stc_soft_timers_t       MotorCtrl_stcSoftTimer;

int32_t     MotorCtrl_i32Q12_IsMaxSquare;
int32_t     MotorCtrl_i32Q8_iqMax;

int32_t     MotorCtrl_i32Q8_SpdPiHighWmHz;
int32_t     MotorCtrl_i32Q16_HighSpdPiKi;
int32_t     MotorCtrl_i32Q10_HighSpdPiKp;
int32_t     MotorCtrl_i32Q16_LowSpdPiKi;
int32_t     MotorCtrl_i32Q10_LowSpdPiKp;

int32_t     MotorCtrl_i32VqMax;

//spd loop
stc_spdLoop_t MotorCtrl_SpdLoopCtrl;


/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/
static void MotorCtrl_InitPar(uint16_t u16SampleFreq);
static void MotorCtrl_SpdReg(void);
static void MotorCtrl_ThetaGen(void);

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/


/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_Init								  									*
*																			  			*
*   Description: Init for motorctrl parameters													*
*																			  			*				
*   Parameters:   u16SampleFreq																*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

void MotorCtrl_Init(uint16_t u16SampleFreq)
{
    MotorCtrl_stcSvmGen.i8ShuntNum = MOTOR_SHUNT_NUM;

    Adc_InitSensorPar(u16SampleFreq);
    
}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_Start								  								*
*																			  			*
*   Description: Init for start parameters														*
*																			  			*				
*   Parameters:   u16SampleFreq																*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

void MotorCtrl_Start(uint16_t u16SampleFreq)
{
	Protector_Init(u16SampleFreq);
	MotorCtrl_InitPar(u16SampleFreq);
	FieldWeaken_Init(u16SampleFreq);        
	MotorStart_Init(&MotorCtrl_stcStartup);
	CV_LimitInit(&CV_LimitPars);        

	MotorCtrl_InitPll(u16SampleFreq);

	// init SVPWM
	stc_svm_cfg_t stc_SvmGenCfg;
	stc_SvmGenCfg.u16Cycle   = MotorCtrl_stcSvmGen.i16Cycle;
	stc_SvmGenCfg.u16DutyMax = MotorCtrl_stcSvmGen.i16DutyMax;
	stc_SvmGenCfg.u16DpwmEn  = 0;
	Svm_InitCalc(&stc_SvmGenCfg,&MotorCtrl_stcSvmCalc);


	if (SENSOR_RUNNING == Adc_u8MotorStatus)
	{	
        Pin_Led_Write(LED_ON);
		PWM_Start();
		MotorCtrl_stcRunPar.u8Status = MOTOR_RUNNING; 
		PWM_Ctrl_Reg_Write(1);       
	}

}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_Stop								  								*
*																			  			*
*   Description: set for stop parameters														*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

//int32_t stop_flag = 0;

void MotorCtrl_Stop(void)
{

    //InitMcu_MotorSvmDis();
    MotorCtrl_stcRunPar.u8Status = MOTOR_STOP;
    MotorCtrl_stcRunPar.i32TargetSpeedRpm = 0;
    MotorCtrl_stcRunPar.i32Q8_EstimWmHz = 0;
    MotorCtrl_stcRunPar.i32Q8_EstimWmHzf = 0;
    MotorCtrl_stcIuvwSensed.i32Q8_Xu = 0;
    MotorCtrl_stcIuvwSensed.i32Q8_Xv = 0;
    MotorCtrl_stcIuvwSensed.i32Q8_Xw = 0;
    
	PWM_Stop();
    Pin_Led_Write(LED_OFF);
    for(uint8_t i = 0;i<64;i++)
    {
        Temp_Speed[i] = 0;
    }
    temp_OutFilter = 0;
	/*prevent running restart*/
	CyDelay(200);

}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_Protect								  								*
*																			  			*
*   Description: Motor Protect function															*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

void MotorCtrl_Protect(void)
{
	
    Protector_LockPar.i32_Edf = MotorCtrl_stcEdq.i32Q8_Xd;//i32_Edf;
    Protector_LockPar.i32_Eqf = MotorCtrl_stcEdq.i32Q8_Xq;//i32_Eqf;

    if(MOTOR_RUNNING == MotorCtrl_stcRunPar.u8Status)
    {

        MotorCtrl_stcRunPar.u32ErroType |= MotorProtect_OutRange(&Protector_OutRangePar,&MotorCtrl_stcIuvwSensed, MotorCtrl_stcRunPar.i32Q8_Vbus);

        if(MotorCtrl_stcRunPar.u8RunningStage < FORCE_STARTUP)
        {			
                //MotorCtrl_stcRunPar.u32ErroType |= MotorProtect_LosePhase1(&Protector_NoneLoadPar,&MotorCtrl_stcIuvwSensed,&MotorCtrl_stcIdqRef);
        }                              
    }
    else
    {
        if(0 != MotorCtrl_stcRunPar.u32ErroType)
        {
                if(++Protector_FlgClr.i32_ErrorFlagHoldTimeCnt
                   > Protector_FlgClr.i32_ErrorFlagHoldTimeNum)
                {
                       // MotorCtrl_stcRunPar.u32ErroType = 0;
                        Protector_FlgClr.i32_ErrorFlagHoldTimeCnt = 0;
                }
        }
    }
}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_InitPar								  								*
*																			  			*
*   Description: Init for same parameters before motor run										*
*																			  			*				
*   Parameters:   u16SampleFreq																*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

static void MotorCtrl_InitPar(uint16_t u16SampleFreq)
{
 
    MotorCtrl_stcRunPar.i32Q8_EstimWmHzf       = 0;
    MotorCtrl_stcRunPar.i32Q22_DeltaThetaKTs   = Q22(1.0*Motor_i32PolePairs/u16SampleFreq);
    MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz  = Q8(Motor_u16ColseLoopTargetSpdHz);
    MotorCtrl_stcRunPar.i32Q22_TargetSpeedWmHz = Q22(Motor_u16ColseLoopTargetSpdHz);
    MotorCtrl_stcRunPar.i32Q22_TargetWmIncTs   = Q22(Motor_f32SpdAccelerationHz * 3.0/u16SampleFreq);
    MotorCtrl_stcRunPar.i32Q22_TargetWmDecTs   = Q22(Motor_f32SpdDecelerationHz * 3.0/u16SampleFreq);
    MotorCtrl_stcRunPar.i32TargetSpeedRpmMax   = Q0(Motor_u16SpdMax);
    MotorCtrl_stcRunPar.i32TargetSpeedRpmMin   = Q0(Motor_u16SpdMin);
    MotorCtrl_stcRunPar.i32Q22_ElecAngle = Q22(0.0);
    MotorCtrl_stcRunPar.u32ErroType      = 0;
    MotorCtrl_stcRunPar.u8SpeedPiEn      = 0;
    MotorCtrl_stcRunPar.u8StartupCompleteFlag = 0;
    MotorCtrl_stcRunPar.u8RunningStage   = MOTOR_ORIENTATION;
    MotorCtrl_stcRunPar.u8Runninglevel   = Motor_u8RunLevel;
    MotorCtrl_stcRunPar.u8CloseloopFlag  = 0;
    MotorCtrl_stcRunPar.u8ChangeSpeedEn  = 0;
    MotorCtrl_stcRunPar.u8ChangePiParEn  = 0;
    MotorCtrl_stcRunPar.u8SpeedPiMode    = 0;
    MotorCtrl_stcSoftTimer.u32Timer0     = 0;
    MotorCtrl_stcSoftTimer.i32Timer1     = 0;
    MotorCtrl_stcSoftTimer.i32Timer2     = 0;
    MotorCtrl_stcSoftTimer.i32Timer3     = 0;
    MotorCtrl_stcIdqRef.i32Q8_Xd = Q8(0.0f);
    MotorCtrl_stcIdqRef.i32Q8_Xq = Q8(0.0f);
    MotorCtrl_stcVabRef.i32Q8_Xa = Q8(0.0f);
    MotorCtrl_stcVabRef.i32Q8_Xb = Q8(0.0f);
    MotorCtrl_stcVdqRef.i32Q8_Xd = Q8(0.0f);
    MotorCtrl_stcVdqRef.i32Q8_Xq = Q8(0.0f);
	

    MotorCtrl_stcSvmGen.i8ShuntNum = MOTOR_SHUNT_NUM;
    MotorCtrl_stcSvmGen.i16T1      = 0;
    MotorCtrl_stcSvmGen.i16T2      = 0;

    MotorCtrl_stcSvmGen.i16AdcTrigT2 = MotorCtrl_stcSvmGen.i16DutyMax - 240;
    MotorCtrl_stcSvmGen.i16AdcTirgT1 = MotorCtrl_stcSvmGen.i16DutyMax >> 1;
    MotorCtrl_stcSvmGen.i16SampleWind = 320;//480;
    MotorCtrl_stcSvmGen.i16SampleOffeset = 100;//220;

    MotorCtrl_i32Q8_iqMax        = Motor_i16Q8_CloseLoopIqRefMax;
    MotorCtrl_i32Q12_IsMaxSquare = (Motor_i16Q8_CloseLoopIsMax
                             * Motor_i16Q8_CloseLoopIsMax) >> 4;

    MotorCtrl_i32Q8_SpdPiHighWmHz = Q8(Motor_u16ChgPiSpdHz);
    MotorCtrl_i32Q16_HighSpdPiKi  = Q16(Motor_f32Ski);
    MotorCtrl_i32Q10_HighSpdPiKp  = Q10(Motor_f32Skp);
    MotorCtrl_i32Q16_LowSpdPiKi   = Q16(Motor_f32LowSpdKi);
    MotorCtrl_i32Q10_LowSpdPiKp   = Q10(Motor_f32LowSpdKp);

    // init speed regulator
  
    MotorCtrl_stcIdPidReg.i32Q15_kp = Q15(Motor_f32Dkp);
    MotorCtrl_stcIdPidReg.i32Q15_ki = Q15(Motor_f32Dki);
    MotorCtrl_stcIdPidReg.i32Q15_kd = Q15(0);
    MotorCtrl_stcIdPidReg.i32_OutMax = Q8(15);//200 15
    MotorCtrl_stcIdPidReg.i32_OutMin = Q8(-15);//-200 -15
    MotorCtrl_stcIdPidReg.i32QN_Iout = 0;
    MotorCtrl_stcIdPidReg.i32_E0  = Q8(0);
    MotorCtrl_stcIdPidReg.i32_E1  = Q8(0);
    MotorCtrl_stcIdPidReg.i32_Iout = Q8(0);
    MotorCtrl_stcIdPidReg.i32_Out  = Q8(0);
    MotorCtrl_stcIdPidReg.i32_OutPre = 0;
    MotorCtrl_stcIdPidReg.i32_Pout = 0;
    MotorCtrl_stcIdPidReg.i32_E0Max = Q8(5); //Q8(10) Q8(5)
    MotorCtrl_stcIdPidReg.i32_E0Min = Q8(-5); //Q8(-10) Q8(-5)
    
    MotorCtrl_stcIqPidReg.i32Q15_kp = Q15(Motor_f32Qkp);
    MotorCtrl_stcIqPidReg.i32Q15_ki = Q15(Motor_f32Qki);
    MotorCtrl_stcIqPidReg.i32Q15_kd = Q15(0);
    MotorCtrl_stcIqPidReg.i32_OutMax = Q8(15);//200 15
    MotorCtrl_stcIqPidReg.i32_OutMin = Q8(0);//-200 0
    MotorCtrl_stcIqPidReg.i32QN_Iout = 0;
    MotorCtrl_stcIqPidReg.i32_E0  = Q8(0);
    MotorCtrl_stcIqPidReg.i32_E1  = Q8(0);
    MotorCtrl_stcIqPidReg.i32_Iout = Q8(0);
    MotorCtrl_stcIqPidReg.i32_Out  = Q8(0);
    MotorCtrl_stcIqPidReg.i32_OutPre = 0;
    MotorCtrl_stcIqPidReg.i32_Pout = 0;
    MotorCtrl_stcIqPidReg.i32_E0Max = Q8(5); //Q8(10) Q8(5)
    MotorCtrl_stcIqPidReg.i32_E0Min = Q8(-5);//Q8(-10) Q8(-5)
   
    MotorCtrl_stcWmPidReg.i32Q15_kp = Q15(Motor_f32LowSpdKp);
    MotorCtrl_stcWmPidReg.i32Q15_ki = Q15(Motor_f32LowSpdKi);
    MotorCtrl_stcWmPidReg.i32Q15_kd = Q15(0);
    MotorCtrl_stcWmPidReg.i32_OutMax = Motor_i16Q8_CloseLoopIqRefMax;
    MotorCtrl_stcWmPidReg.i32_OutMin = 0;
    MotorCtrl_stcWmPidReg.i32QN_Iout = 0;
    MotorCtrl_stcWmPidReg.i32_E0  = Q8(0);
    MotorCtrl_stcWmPidReg.i32_E1  = Q8(0);
    MotorCtrl_stcWmPidReg.i32_Iout = Q8(0);
    MotorCtrl_stcWmPidReg.i32_Out  = Q8(0);
    MotorCtrl_stcWmPidReg.i32_OutPre = 0;
    MotorCtrl_stcWmPidReg.i32_Pout = 0;
    MotorCtrl_stcWmPidReg.i32_Dout = 0;
    MotorCtrl_stcWmPidReg.i32_E0Max = Q8(10);//100  10
    MotorCtrl_stcWmPidReg.i32_E0Min = Q8(-10);//-100  -10
    MotorCtrl_stcWmPidReg.I_cnt = 0;
    MotorCtrl_stcWmPidReg.I_timer = 1;
    
    MotorCtrl_SpdLoopCtrl.i32_LpfMinSpd = Q8(65);
    MotorCtrl_SpdLoopCtrl.i32_LpdMaxSpd = Q8(100);
    MotorCtrl_SpdLoopCtrl.i32Q12_LpfMax = Q12(1.0);
    MotorCtrl_SpdLoopCtrl.i32Q12_LpfMin = Q12(0.1);
    if(MotorCtrl_SpdLoopCtrl.i32_LpfMinSpd 
       != MotorCtrl_SpdLoopCtrl.i32_LpdMaxSpd)
    {
       MotorCtrl_SpdLoopCtrl.i32Q8_LpfDelta 
         =  Q8((MotorCtrl_SpdLoopCtrl.i32Q12_LpfMax - MotorCtrl_SpdLoopCtrl.i32Q12_LpfMin))
             /(MotorCtrl_SpdLoopCtrl.i32_LpdMaxSpd - MotorCtrl_SpdLoopCtrl.i32_LpfMinSpd);
       if(MotorCtrl_SpdLoopCtrl.i32Q8_LpfDelta < 0)
       {
          MotorCtrl_SpdLoopCtrl.i32Q8_LpfDelta = 0;
       }
    }
    
    MotorCtrl_SpdLoopCtrl.i32Q8_SpdOut = 0;
    MotorCtrl_SpdLoopCtrl.i32_SpdLoopCnt = 0;
    MotorCtrl_SpdLoopCtrl.i32_SpdLoopCyl = 2; //5
    MotorCtrl_SpdLoopCtrl.SpdOut_lpf.i32Q12_LPFK = Q12(0.3); //Q12(0.3)
    MotorCtrl_SpdLoopCtrl.SpdOut_lpf.i32Q20_LowBits = 0;

}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_SpdReg								  								*
*																			  			*
*   Description: motor speed regulator															*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                               				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

static void MotorCtrl_SpdReg(void)
{
    MotorCtrl_SpdLoopCtrl.i32_SpdLoopCnt ++;
    if(MotorCtrl_SpdLoopCtrl.i32_SpdLoopCnt > MotorCtrl_SpdLoopCtrl.i32_SpdLoopCyl)
    {
        MotorCtrl_SpdLoopCtrl.i32_SpdLoopCnt = 0;
    }
    int32_t Q8_TargetSpeed_Hz_temp;
    Filter_OneOderLpf(MotorCtrl_stcRunPar.i32Q8_EstimWmHz, &MotorCtrl_stcRunPar.i32Q8_EstimWmHzf, &MotorCtrl_stc1stLpfWm);
   //EstHzFlag = MotorCtrl_stcRunPar.i32Q8_EstimWmHz;
    /********************************Motor Target Speed Regulate***************************/
    if (MotorCtrl_stcRunPar.u8ChangeSpeedEn == 1)
    {   
        if(0 != MotorCtrl_stcRunPar.i32TargetSpeedRpm)
        {            
            if (MotorCtrl_stcRunPar.i32TargetSpeedRpm > MotorCtrl_stcRunPar.i32TargetSpeedRpmMax)
            {
                    MotorCtrl_stcRunPar.i32TargetSpeedRpm = MotorCtrl_stcRunPar.i32TargetSpeedRpmMax;
            }
            else if(MotorCtrl_stcRunPar.i32TargetSpeedRpm< MotorCtrl_stcRunPar.i32TargetSpeedRpmMin)
            {
                    MotorCtrl_stcRunPar.i32TargetSpeedRpm = MotorCtrl_stcRunPar.i32TargetSpeedRpmMin;
            }         
            Q8_TargetSpeed_Hz_temp = (MotorCtrl_stcRunPar.i32TargetSpeedRpm<<8)/60;
            if ((Q8_TargetSpeed_Hz_temp - MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz) > (MotorCtrl_stcRunPar.i32Q22_TargetWmIncTs >> 14))
            {
                
                MotorCtrl_stcRunPar.i32Q22_TargetSpeedWmHz += MotorCtrl_stcRunPar.i32Q22_TargetWmIncTs;
                MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz = MotorCtrl_stcRunPar.i32Q22_TargetSpeedWmHz >> 14;
            }
            else if ((Q8_TargetSpeed_Hz_temp - MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz) < -(MotorCtrl_stcRunPar.i32Q22_TargetWmDecTs
                    >> 14))
            {
                
                MotorCtrl_stcRunPar.i32Q22_TargetSpeedWmHz -= MotorCtrl_stcRunPar.i32Q22_TargetWmDecTs;
                MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz = MotorCtrl_stcRunPar.i32Q22_TargetSpeedWmHz >> 14;
            }
            else
            {
               // lpfen = 1;
                MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz = Q8_TargetSpeed_Hz_temp;
            }
        }
    }
	
    /*******************************Motor Speed Pi Regulate*******************************/ 
	#if 0
    if (MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz > MotorCtrl_i32Q8_SpdPiHighWmHz)
    {
		if (MotorCtrl_stcWmPidReg.i32Q15_kp < (MotorCtrl_i32Q10_HighSpdPiKp<<5))
		{
			MotorCtrl_stcWmPidReg.i32Q15_kp += KP_STEP;
		}

		if (MotorCtrl_stcWmPidReg.i32Q15_ki < (MotorCtrl_i32Q16_HighSpdPiKi>>1))
		{
			MotorCtrl_stcWmPidReg.i32Q15_ki += KI_STEP;
		}
		//MotorCtrl_stcWmPidReg.i32Q15_kp = MotorCtrl_i32Q10_HighSpdPiKp<<5;
		//MotorCtrl_stcWmPidReg.i32Q15_ki = MotorCtrl_i32Q16_HighSpdPiKi>>1;
    }
    else if (MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz < MotorCtrl_i32Q8_SpdPiHighWmHz - Q8(5))
    {
		MotorCtrl_stcWmPidReg.i32Q15_kp = MotorCtrl_i32Q10_LowSpdPiKp<<5;
		MotorCtrl_stcWmPidReg.i32Q15_ki = MotorCtrl_i32Q16_LowSpdPiKi>>1;         
    }
    #else
	MotorCtrl_stcWmPidReg.i32Q15_kp = MotorCtrl_i32Q10_HighSpdPiKp<<5;
	MotorCtrl_stcWmPidReg.i32Q15_ki = MotorCtrl_i32Q16_HighSpdPiKi>>1; 
	#endif

	#if 0
    /***********SpdLoop out lpf regulate**************/
    if(MotorCtrl_stcRunPar.i32Q8_EstimWmHzf > MotorCtrl_SpdLoopCtrl.i32_LpfMinSpd)
    {
        MotorCtrl_SpdLoopCtrl.SpdOut_lpf.i32Q12_LPFK 
          = Q12(1) - (((MotorCtrl_stcRunPar.i32Q8_EstimWmHzf - MotorCtrl_SpdLoopCtrl.i32_LpfMinSpd)
            *MotorCtrl_SpdLoopCtrl.i32Q8_LpfDelta) >> 8);
        if(MotorCtrl_SpdLoopCtrl.SpdOut_lpf.i32Q12_LPFK < MotorCtrl_SpdLoopCtrl.i32Q12_LpfMin)
        {
            MotorCtrl_SpdLoopCtrl.SpdOut_lpf.i32Q12_LPFK =  MotorCtrl_SpdLoopCtrl.i32Q12_LpfMin;
        } 
        if(lpfen == 0)
        {
            MotorCtrl_SpdLoopCtrl.SpdOut_lpf.i32Q12_LPFK = Q12(1);
        }
    }
	#endif
	
    /*******************************Motor PI Parameter Regulate***************************/
    if (MotorCtrl_stcRunPar.u8SpeedPiEn == SPEEDPI_RUNNING 
        && MotorCtrl_SpdLoopCtrl.i32_SpdLoopCnt == MotorCtrl_SpdLoopCtrl.i32_SpdLoopCyl)
    {
        MotorCtrl_SpdLoopCtrl.i32_SpdLoopCnt = 0; 

		Pid_Pos(&MotorCtrl_stcWmPidReg,MotorCtrl_stcRunPar.i32Q8_TargetSpeedWmHz-MotorCtrl_stcRunPar.i32Q8_EstimWmHzf);
		
        MotorCtrl_SpdLoopCtrl.i32Q8_SpdOut = MotorCtrl_stcWmPidReg.i32_Out;
        Filter_OneOderLpf(MotorCtrl_SpdLoopCtrl.i32Q8_SpdOut, &MotorCtrl_stcIdqRef.i32Q8_Xq, &MotorCtrl_SpdLoopCtrl.SpdOut_lpf);
    } 

}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_ThetaGen								  							*
*																			  			*
*   Description: motor angle generate															*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

static void MotorCtrl_ThetaGen(void)
{
	uint32_t ui32_temp;
	
	if (MotorCtrl_stcRunPar.u8CloseloopFlag == 2)
	{
	    MotorCtrl_stcRunPar.i32Q8_EstimWmHz = Motor_stcPll.i32Q8_EstimWmHz;
	    MotorCtrl_stcRunPar.i32Q22_ElecAngle = (int32_t) (Motor_stcPll.i32Q22_ElecAngle);
	    MotorCtrl_stcRunPar.i32Q22_DeltaThetaTs = Motor_stcPll.i32Q22_DeltaThetaTs;
	}
    else if (MotorCtrl_stcRunPar.u8CloseloopFlag == 0)
    {
		MotorCtrl_stcRunPar.i32Q8_EstimWmHz = Motor_stcPll.i32Q8_EstimWmHz;
		MotorCtrl_stcRunPar.i32Q22_ElecAngle += MotorCtrl_stcRunPar.i32Q22_DeltaThetaTs;
    }

    MotorCtrl_stcRunPar.i32Q22_ElecAngle &= 0x3FFFFF;
    
    ui32_temp = MotorCtrl_stcRunPar.i32Q22_ElecAngle>>12; 
	
    MotorCtrl_stcVdqRef.i32Q12_Sin = Math_Sin(ui32_temp);  
    MotorCtrl_stcVdqRef.i32Q12_Cos = Math_Cos(ui32_temp);
	
    MotorCtrl_stcIdqSensed.i32Q12_Sin = MotorCtrl_stcVdqRef.i32Q12_Sin;
    MotorCtrl_stcIdqSensed.i32Q12_Cos = MotorCtrl_stcVdqRef.i32Q12_Cos;

	#if (0)
    MotorCtrl_stcIdqRef.i32Q12_Sin = MotorCtrl_stcVdqRef.i32Q12_Sin;
    MotorCtrl_stcIdqRef.i32Q12_Cos = MotorCtrl_stcVdqRef.i32Q12_Cos;
	#endif
	
    MotorCtrl_stcEdq.i32Q12_Cos = MotorCtrl_stcVdqRef.i32Q12_Cos;
    MotorCtrl_stcEdq.i32Q12_Sin = MotorCtrl_stcVdqRef.i32Q12_Sin;
}

/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_WriteOCCP								  							*
*																			  			*
*   Description: PWM output																	*
*																			  			*				
*   Parameters:   stc_svm_gen_t																*
*				  							  			                               				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

void MotorCtrl_WriteOCCP(stc_svm_gen_t *pstcVal)
{
if(Motor_u8RunningDirection == CW)
{
	PWMU_WriteCompareBuf(pstcVal->i16Aon);
    PWMV_WriteCompareBuf(pstcVal->i16Bon);
    PWMW_WriteCompareBuf(pstcVal->i16Con);
}
else
{
	PWMU_WriteCompareBuf(pstcVal->i16Bon);
    PWMV_WriteCompareBuf(pstcVal->i16Aon);
    PWMW_WriteCompareBuf(pstcVal->i16Con);

}
}


/****************************************************************************************
*																			  			*
*   Function:    MotorCtrl_Process								  								*
*																			  			*
*   Description: Motor control main flow														*
*																			  			*				
*   Parameters:   None																		*
*				  							  			                                				*
*   Returns:     None														  				*		
*																			  			*		
****************************************************************************************/

void MotorCtrl_Process(void)
{
    
	MotorCtrl_stcSoftTimer.i32Timer3++;
    /* Protection*/
    MotorCtrl_Protect(); 
	
	if (MotorCtrl_stcRunPar.u8Status == MOTOR_RUNNING)
	{  
        /*Wait for ADC Sample Complete*/
	    while (Adc_stcSample.u8_CompleteFlag != 1)
		{
			;
		}
		
		Adc_stcSample.u8_CompleteFlag = 0;
		
		/* Read ADC and Get phase current*/
		Adc_MotorCurrentSense();

        /*Clark and Park Transformation*/
		Clark(&MotorCtrl_stcIuvwSensed, &MotorCtrl_stcIabSensed); 
		Park(&MotorCtrl_stcIabSensed, &MotorCtrl_stcIdqSensed);

			/*Position and Speed Estimate*/	
			MotorPll_PostionEstimate(&Motor_stcPll,&MotorCtrl_stcVabReal,&MotorCtrl_stcIabSensed); 

			/**Rotor lock protection function*/			
#if 1
			MotorCtrl_stcEab.i32Q8_Xa = Motor_stcPll.i32Q8_Ealpha;
			MotorCtrl_stcEab.i32Q8_Xb = Motor_stcPll.i32Q8_Ebeta;
			Park(&MotorCtrl_stcEab, &MotorCtrl_stcEdq);
			
	   		MotorCtrl_stcRunPar.u32ErroType |= MotorProtect_MotorLock(&Protector_LockPar);
#endif

			MotorCtrl_stcSoftTimer.i32Timer3 &= 0x3;
			if (0 == MotorCtrl_stcSoftTimer.i32Timer3)
			{
                /*Speed Loop*/
				MotorCtrl_SpdReg(); 		 
			}
			else if (1 == MotorCtrl_stcSoftTimer.i32Timer3)
			{
                /*Field Weaken*/
				#if 1
			   	MotorFw_FieldWeaken(&FieldWeaken_Ctrl,
								   &MotorCtrl_i32VqMax,
								   &MotorCtrl_stcIdqRef.i32Q8_Xd,
								   MotorCtrl_stcRunPar.u8RunningStage); 
				#endif
			}
			else if (2 == MotorCtrl_stcSoftTimer.i32Timer3)
			{
                /*Calculate the Limitation of Current and Voltage*/
				CV_LimitCtrl();
			}	
		
			if(MotorCtrl_stcRunPar.u32ErroType != 0)
			{
				MotorCtrl_Stop();
			}
	        /*Current Loop*/
			Pid_Pos(&MotorCtrl_stcIdPidReg,MotorCtrl_stcIdqRef.i32Q8_Xd-MotorCtrl_stcIdqSensed.i32Q8_Xd);					
			MotorCtrl_stcVdqRef.i32Q8_Xd = MotorCtrl_stcIdPidReg.i32_Out;		  
			Pid_Pos(&MotorCtrl_stcIqPidReg,MotorCtrl_stcIdqRef.i32Q8_Xq-MotorCtrl_stcIdqSensed.i32Q8_Xq);	   
			MotorCtrl_stcVdqRef.i32Q8_Xq = MotorCtrl_stcIqPidReg.i32_Out;	
			
#if 1
			if(MotorCtrl_stcVdqRef.i32Q8_Xq > MotorCtrl_i32VqMax)
			{
				MotorCtrl_stcVdqRef.i32Q8_Xq = MotorCtrl_i32VqMax;
			}
#endif
            /*Calculate Angle*/
			MotorCtrl_ThetaGen();

#if 1
            /*Motor Startup Stage*/
			if(FALSE == MotorCtrl_stcRunPar.u8StartupCompleteFlag)
			{ 	 
				MotorCtrl_stcRunPar.u8StartupCompleteFlag 			   
				  = MotorStartup(&MotorCtrl_stcStartup); 
				MotorCtrl_stcIdqRef.i32Q8_Xq = (MotorCtrl_stcStartup.i32Q12_Iq)>>4;						   
			}
#endif
	        /* Inverse Park Transformation*/
			InvPark(&MotorCtrl_stcVdqRef, &MotorCtrl_stcVabRef);	  
		
        /* SVPWM Calculation*/
		MotorCtrl_stcSvmCalc.i32QN_VaIn = MotorCtrl_stcVabRef.i32Q8_Xa;
		MotorCtrl_stcSvmCalc.i32QN_VbIn = MotorCtrl_stcVabRef.i32Q8_Xb;
		MotorCtrl_stcSvmCalc.i32QN_VdcIn = MotorCtrl_stcRunPar.i32Q8_Vbus;
		            
		Svm_Calc(&MotorCtrl_stcSvmCalc);
		  
		MotorCtrl_stcSvmGen.i16Aon = MotorCtrl_stcSvmCalc.u16Uon;
		MotorCtrl_stcSvmGen.i16Bon = MotorCtrl_stcSvmCalc.u16Von;
		MotorCtrl_stcSvmGen.i16Con = MotorCtrl_stcSvmCalc.u16Won;
		MotorCtrl_stcVabReal.i32Q8_Xa = MotorCtrl_stcSvmCalc.i32QN_VaReal;
		MotorCtrl_stcVabReal.i32Q8_Xb = MotorCtrl_stcSvmCalc.i32QN_VbReal;

		MotorCtrl_stcSvmGen.i8SectorPre = MotorCtrl_stcSvmGen.i8Sector;
		MotorCtrl_stcSvmGen.i8Sector = MotorCtrl_stcSvmCalc.u16Sector;

#if 1
		if(Adc_stcSample.u8_CompleteFlag == 1)
		{
			Adc_stcSample.u8_CompleteFlag = 0;
		}
#endif
        /* Update PWM Duty*/
		MotorCtrl_WriteOCCP(&MotorCtrl_stcSvmGen);
        
    }  
}

