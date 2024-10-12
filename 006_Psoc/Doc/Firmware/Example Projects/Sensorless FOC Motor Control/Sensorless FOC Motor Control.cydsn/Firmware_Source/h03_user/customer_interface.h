/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: customer_interface.h
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

#ifndef __CUSTOMER_INTERFACE_H__
#define __CUSTOMER_INTERFACE_H__

/*****************************************************************************/
/* Include files                                                             */
/*****************************************************************************/
#include "define.h"

/*****************************************************************************/
/* Global pre-processor symbols/macros ('#define')                           */
/*****************************************************************************/

/*****************************************************************************/
/* Global type definitions ('typedef')                                       */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable declarations ('extern', definition in C source)           */
/*****************************************************************************/

/******************************************************************************
 * HW configuration - PFC and motor
 ******************************************************************************/

/******************************************************************************
 * switch parameters - PFC and motor
 ******************************************************************************/

extern uint8_t   Motor_u8FieldWeakenEn;      // 1: enable
                                       // 0: disable

/******************************************************************************
 * control parameters - PFC
 ******************************************************************************/


/******************************************************************************
 * motor manufacture parameters - motor
 ******************************************************************************/

extern int32_t Motor_i32PolePairs;      //the pole pairs of rotor
extern float32_t Motor_f32Ld;   // the d axis inductance
extern float32_t Motor_f32Lq;	  // the q axis inductance
extern float32_t Motor_f32Res;  // the resistance between two phases
extern float32_t Motor_f32Ke;  // inductive voltage constant between two phases
extern float32_t Motor_f32FwIdMax;           //Nm/A

/******************************************************************************
 * Motor running parameters - motor
 ******************************************************************************/

//
// sampling
//

extern float32_t Motor_f32IuvwSampleResistor;   // Iuvw sample resistor (ohm)
extern float32_t Motor_i32IuvwAmplifierFactor;  // Vdc calculation factor
extern int32_t   Motor_i32IuvwOffsetNormal;
extern int32_t   Motor_i32IuvwOffsetRange;      // ADC offset range of Iuvw sampling
extern uint32_t  Motor_u32IuvwOffsetCheckTimes; // Iuvw ADC sample offset check



//
// Dead time, carrier frequency, direction and speed
//
extern float32_t Motor_f32DeadTimeMicroSec;  //dead timer us
extern uint16_t  Motor_u16CarrierFreq;       // motor carry frequency (hz)
                                            // Range: [5(khz), 12(khz)]
extern float32_t Motor_f32SpdAccelerationHz; // acceleration speed
extern float32_t Motor_f32SpdDecelerationHz; // deceleration speed

//
// PID
//
extern float32_t Motor_f32LowSpdKi;    //speed PI regulator integral constant
extern float32_t Motor_f32LowSpdKp;    //speed PI regulator proportion constant
extern float32_t Motor_f32Dki;         //d axis current PI regulator integral constant
extern float32_t Motor_f32Dkp;         //d axis current PI regulator proportion constant
extern float32_t Motor_f32Qki;         //q axis current PI regulator integral constant
extern float32_t Motor_f32Qkp;         //q axis current PI regulator proportion constant
extern float32_t Motor_f32Ski;         //speed PI regulator integral constant
extern float32_t Motor_f32Skp;         //speed PI regulator proportion constant
extern uint16_t  Motor_u16ChgPiSpdHz;  //PID parameters change
                                      //at this speed


//
// running control of open loop
//
extern uint8_t   Motor_u8RunLevel; // 1->orientation,
                                  // 2->open loop running,
                                  // 3->closed loop running,
                                  // 4->change speed enable

extern int16_t   Motor_i16Q8_OrientEndIqRef;    // orientation current
extern int16_t	 Motor_i16Q8_OrientInitIqRef;

extern float32_t Motor_f32OrientIqrefIncAPS;
extern float32_t   Motor_f32OrientTime;
extern uint16_t  Motor_u16OpenLoopSpdIncHz;
extern uint16_t  Motor_u16OpenLoopSpdInitHz;
extern uint16_t  Motor_u16OpenLoopSpdEndHz;

extern int16_t   Motor_i16Q8_OpenLoopInitIqRef;
extern int16_t   Motor_i16Q8_OpenLoopEndIqRef;  // q axis current reference
                                            // in open loop stage
extern float32_t Motor_f32OpenLoopIqrefIncAPS;

//
// running control of closed loop
//
extern uint16_t  Motor_u16ColseLoopTargetSpdHz;
extern int16_t   Motor_i16Q8_CloseLoopIsMax;
extern int16_t   Motor_i16Q8_CloseLoopIqRefMax; // the maximum value of q
                                            // axis current reference
                                            // in closed loop



//
// Field weakening
//
extern float32_t Motor_f32FwStart;

//
// torque compensation
//


//
// protection
//
extern int16_t  Motor_u16SpdMax;          // motor run maximum speed rpm
extern int16_t  Motor_u16SpdMin;          // motor run minimum speed rpm
extern int16_t   Motor_i16Q8_CurrentMax;    // motor phase current peak A
extern uint16_t  Motor_u16OverCurrentTimeSec; // motor over current timer PWM
extern uint16_t  Motor_u16VbusMax;         // the maximum value of DC
                                          // voltage is 48V
extern uint16_t  Motor_u16VbusMin;         // the minimum value of DC
                                          // voltage is 24V

extern uint8_t  Motor_ErrorTime; 
extern uint8_t  Motor_u8RunningDirection;

extern int8_t    Motor_u8BrakeCtrlEn;
extern uint16_t  Motor_u16BrakeTime;


extern uint8_t	 Motor_u8SpeedSetEn;


/*****************************************************************************/
/* Global function prototypes ('extern', definition in C source)             */
/*****************************************************************************/

#endif /* __CUSTOMER_INTERFACE_H__ */



