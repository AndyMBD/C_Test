/*******************************************************************************
* Project Name		: Sensorless FOC Motor Control
* File Name			: customer_interface.c
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
#include "h03_user\customer_interface.h"
#include "define.h"

/*****************************************************************************/
/* Local pre-processor symbols/macros ('#define')                            */
/*****************************************************************************/

/*****************************************************************************/
/* Global variable definitions (declared in header file with 'extern')       */
/*****************************************************************************/

/** UI_01  define the used motor parameter in this project*/ 
int32_t   Motor_i32PolePairs        = 4;   	// the pole pairs of rotor
float32_t Motor_f32Ld             = 0.006;  // the d axis inductance   
float32_t Motor_f32Lq             = 0.006;	// the q axis inductance  
float32_t Motor_f32Res            = 1.1;    // the resistance between two phases
float32_t Motor_f32Ke             = 2.83;   // inductive voltage constant between two phases,RMS value


/** UI_03  define the hardware's a/d sample information, carrier frequency and dead time*/
float32_t Motor_f32IuvwSampleResistor    = 0.03; // Iuvw sample resistor (ohm)
float32_t Motor_i32IuvwAmplifierFactor   = 4.16; // Iuvw calculation factor

int32_t   Motor_i32IuvwOffsetNormal      = 2048;   // the middle value of 12-bits ADC
int32_t   Motor_i32IuvwOffsetRange       = 150;    // ADC offset range of Iuvw sampling
uint32_t   Motor_u32IuvwOffsetCheckTimes  = 64;    // Iuvw ADC sample offset
                                                   // check times
float32_t Motor_f32DeadTimeMicroSec      = 0.8f;   // Dead timer us
uint16_t  Motor_u16CarrierFreq           = 10000;  // motor carry frequency (hz)
                                                   // Range: [5(khz), 12(khz)]

float32_t Motor_f32Dki				   = 0.06f;	 	//d axis current PI regulator integral constant 
float32_t Motor_f32Dkp				   = 0.60f;		//d axis current PI regulator proportion constant
float32_t Motor_f32Qki				   = 0.06f;	 	//q axis current PI regulator integral constant
float32_t Motor_f32Qkp				   = 0.60f;		//q axis current PI regulator proportion constant

float32_t Motor_f32LowSpdKi			   = 0.005f; 	//speed PI regulator integral constant
float32_t Motor_f32LowSpdKp			   = 0.15f;		//speed PI regulator proportion constant
float32_t Motor_f32Ski				   = 0.005f;    //speed PI regulator integral constant  
float32_t Motor_f32Skp				   = 0.15; 		//speed PI regulator proportion constant


uint16_t  Motor_u16ChgPiSpdHz            = 30;      //PID parameters change at this speed

/** UI_05  Configure startup parameters */
uint8_t   Motor_u8RunLevel              = 4;    // 1->orientation,
                                                // 2->open loop running,
                                                // 3->closed loop running,
                                                // 4->change speed enable
int16_t   Motor_i16Q8_OrientEndIqRef       = Q8(0.5); // orientation current
int16_t   Motor_i16Q8_OrientInitIqRef   = Q8(0);      //orientation initial current
float32_t Motor_f32OrientIqrefIncAPS   = 1.0f;        //orientation current added per sec
float32_t Motor_f32OrientTime         = 0.0f;         // orientation time,Second
 
uint16_t  Motor_u16OpenLoopSpdInitHz    = 3; 	    //force start initial speed
uint16_t  Motor_u16OpenLoopSpdEndHz     = 3;	    //force start end speed
uint16_t  Motor_u16OpenLoopSpdIncHz     = 3;	    //force start speed added per sec

int16_t   Motor_i16Q8_OpenLoopInitIqRef = Q8(1.0);    // force start q axis initial current reference 
int16_t   Motor_i16Q8_OpenLoopEndIqRef     = Q8(2.0); //force start q axis current reference 
float32_t Motor_f32OpenLoopIqrefIncAPS   = 2.0f;      //force start current added per sec
//APS

/** UI_06  close-loop running parameters setting*/
uint16_t  Motor_u16ColseLoopTargetSpdHz = 10; //3
uint8_t   Motor_u8RunningDirection    = CW;         //0: CW, 1:CCW

int16_t   Motor_i16Q8_CloseLoopIsMax    = Q8(6.0);
int16_t   Motor_i16Q8_CloseLoopIqRefMax = Q8(5.0);// the maximum value of q
                                                  // axis current reference
                                                  // in closed loop
int16_t   Motor_u16SpdMax                = 4000;   // motor run maximum speed rpm 
int16_t   Motor_u16SpdMin                = 500;    // motor run minimum speed rpm 
float32_t Motor_f32SpdAccelerationHz     = 100.0f; // acceleration speed 
float32_t Motor_f32SpdDecelerationHz     = 100.0f; // deceleration speed 

/** UI_07  protection parameters setting*/
int16_t   Motor_i16Q8_CurrentMax       = Q8(7); // motor phase current peak A
uint16_t  Motor_u16OverCurrentTimeSec = 1;      // motor over current timer PWM
uint16_t  Motor_u16VbusMax            = 34;     // the maximum value of DC
                                                // voltage 
uint16_t  Motor_u16VbusMin            = 20;     // the minimum value of DC
                                                // voltage 
uint8_t   Motor_ErrorTime             = 10;     //s

/** UI_08  brake parameters setting*/
int8_t    Motor_u8BrakeCtrlEn           = 0;    // 1: enable brake control
                                                // 0: disable brake control
uint16_t  Motor_u16BrakeTime     		 = 1;   //brake time ms                                           

/** UI_09  field weaken parameters setting*/
uint8_t   Motor_u8FieldWeakenEn                 = 0;  // 1: enable
                                                      // 0: disable
float32_t Motor_f32FwStart              = 1.0f;    
float32_t Motor_f32FwIdMax              = 5.0; 
/** UI_10  enable speed set parameters setting*/

uint8_t	  Motor_u8SpeedSetEn     	 = 1;


/*****************************************************************************/
/* Local type definitions ('typedef')                                        */
/*****************************************************************************/

/*****************************************************************************/
/* Local variable definitions ('static')                                     */
/*****************************************************************************/

/*****************************************************************************/
/* Local function prototypes ('static')                                      */
/*****************************************************************************/

/*****************************************************************************/
/* Function implementation - global ('extern') and local ('static')          */
/*****************************************************************************/

