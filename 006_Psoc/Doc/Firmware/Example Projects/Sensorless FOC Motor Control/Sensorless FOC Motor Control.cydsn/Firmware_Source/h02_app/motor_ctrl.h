/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: motor_ctrl.h
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

#ifndef __COMP_CTRL_H__
#define __COMP_CTRL_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "h01_module\coordinate_transform.h"
#include "h03_user\customer_interface.h"
#include "h02_app\field_weaken.h"
#include "h02_app\adc_sample.h"
#include "h01_module\pid_regulator.h"
#include "h01_module\svpwm.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

#define CW                     0 // motor run direction - clockwise
#define CCW                    1 // motor run direction - counter-clockwise


#define MOTOR_STOP             0
#define MOTOR_RUNNING          1

#define SPEEDPI_STOP           0
#define SPEEDPI_RUNNING        1

#define BRAKE                  0
#define PREHOT                 0
#define ORIENTE                1
#define FORCE_STARTUP          2
#define ENTER_CLOSEDLOOP       3
#define CHANGE_SPEED           4

#define NORMAL_RUNNING       0x00
#define OVER_VOLTAGE         0x01   /*Over Voltage Protection*/
#define UNDER_VOLTAGE        0x02   /*Under Voltage Protection*/
#define SW_OVER_CURRENT      0x04   /*Phase Over Current Protection*/
#define MOTOR_OVER_CURRENT   0x08   /*Bus Over Current Protection*/
#define MOTOR_LOSE_PHASE     0x10
#define NO_CONECT_COMPRESSOR 0x20
#define AD_MIDDLE_ERROR      0x40   /*AD Middle Error Protection*/
#define SF_WTD_RESET         0x80
#define MOTOR_LOCK           0x100  /*Motor Lock Protection*/
#define UNDEFINED_INT        0x200
#define HW_WTD_RESET         0x400

#define SENSOR_STOP          0
#define SENSOR_RUNNING       1

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/
//
// motor running
//
typedef struct
{
     
    volatile int32_t  i32TargetSpeedRpm;/*Motor Target Speed*/
    int32_t  i32TargetSpeedRpmMax;      /*The Max of Motor Target Speed*/
    int32_t  i32TargetSpeedRpmMin;      /*The Min of Motor Target Speed*/
    int32_t  i32Q8_EstimWmHz;           /*Estimated speed in Q8 Format, Unit:Hz*/
    int32_t  i32Q8_EstimWmHzf;          /*Filtered Estimated Speed in Q8 Format, Unit:Hz*/
     uint8_t u8Status;                  /*Running Status, 0:Stop, 1:Running*/
     uint32_t u32ErroType;              /*Protect Type*/
     int32_t i32Q8_VdcReciprocal;
     int32_t i32Q8_Vbus;                /*Bus Voltage in Q8 Format*/
	 int32_t i32Q8_VR;
    int32_t  i32Q22_DeltaThetaTs;
    int32_t  i32Q22_DeltaThetaKTs;
    int32_t  i32Q8_TargetSpeedWmHz;
    int32_t  i32Q22_TargetSpeedWmHz;
    int32_t  i32Q22_TargetWmIncTs;
    int32_t  i32Q22_TargetWmDecTs;
    int32_t  i32Q22_ElecAngle;
    int32_t  i32Q22_ElecAngleErro;
    uint8_t  u8SpeedPiEn;
    uint8_t  u8StartupCompleteFlag;
    uint8_t  u8RunningStage;
    uint8_t  u8Runninglevel;
    uint8_t  u8CloseloopFlag;           /*Flag of Close Loop,0:Open Loop, 1: Close Loop*/
    uint8_t  u8ChangeSpeedEn;
    uint8_t  u8ChangePiParEn;
    uint8_t  u8SpeedPiMode;
    uint32_t u32StartPiParValidTime;

} stc_motor_run_t;

typedef struct
{
    volatile uint32_t u32Timer0;
    volatile uint32_t i32Timer1;
    volatile uint32_t i32Timer2;
    volatile uint32_t i32Timer3;
} stc_soft_timers_t;

typedef struct
{
     int32_t i32_SpdLoopCnt;
     int32_t i32_SpdLoopCyl;
     int32_t i32_LpfMinSpd;
     int32_t i32_LpdMaxSpd;
     int32_t i32Q12_LpfMin;
     int32_t i32Q12_LpfMax;
     int32_t i32Q8_LpfDelta;
     int32_t  i32Q8_SpdOut;
     stc_one_order_lpf_t SpdOut_lpf;
} stc_spdLoop_t;

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/
// sample
extern stc_uvw_t       MotorCtrl_stcIuvwSensed;

// coordinate transformation
extern stc_ab_t        MotorCtrl_stcIabSensed;
extern stc_dq_t        MotorCtrl_stcIdqSensed;
extern stc_dq_t        MotorCtrl_stcIdqRef;
extern stc_dq_t        MotorCtrl_stcEdq;


extern stc_dq_t        MotorCtrl_stcVdqRef;
extern stc_ab_t        MotorCtrl_stcVabRef;
extern stc_uvw_t       MotorCtrl_stcVuvwRef;

// PID
extern stc_pid_cfg_t       MotorCtrl_stcIqPidReg;
extern stc_pid_cfg_t       MotorCtrl_stcIdPidReg;
extern stc_pid_cfg_t       MotorCtrl_stcWmPidReg;

// PWM
extern stc_svm_gen_t   MotorCtrl_stcSvmGen;
extern stc_svm_calc_t  MotorCtrl_stcSvmCalc;

// motor ctrl
extern stc_motor_run_t  MotorCtrl_stcRunPar;
extern stc_soft_timers_t       MotorCtrl_stcSoftTimer;
extern stc_one_order_lpf_t     MotorCtrl_stc1stLpfWm;

extern int32_t MotorCtrl_i32Q12_IsMaxSquare;
extern int32_t MotorCtrl_i32Q8_iqMax;

extern int32_t MotorCtrl_i32Q8_SpdPiHighWmHz;
extern int32_t MotorCtrl_i32Q16_HighSpdPiKi;
extern int32_t MotorCtrl_i32Q10_HighSpdPiKp;
extern int32_t MotorCtrl_i32Q16_LowSpdPiKi;
extern int32_t MotorCtrl_i32Q10_LowSpdPiKp;

extern int32_t MotorCtrl_i32VqMax;
//extern int32_t stop_flag;

/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/


extern void PWM_OutputEnable(void);

extern void PWM_OutputDisable(void);

extern void MotorCtrl_Stop(void);

extern void MotorCtrl_Init(uint16_t u16SampleFreq);

extern void MotorCtrl_Process(void);

extern void MotorCtrl_Start(uint16_t u16SampleFreq);

extern void MotorCtrl_Stop(void);

#endif /* __COMP_CTRL_H__ */


