/*******************************************************************************
* Project Name		: Sensorless BLDC Motor Control
* File Name			: Parameters.h
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
#ifndef __PARAMETER_H__
#define __PARAMETER_H__

/******************************************************************************
 * Header file including
 ******************************************************************************/ 
#include <cytypes.h>

/******************************************************************************
 * Macro declaration   
 ******************************************************************************/  
/*============================================================================   
 * macro for motor physical parameters
 *============================================================================*/      
/* motor pole-pairs number */
#define MOTOR_POLE_PAIR_NUM                 ((uint8)(4))     
/*============================================================================   
 * macro for solution general setting
 *============================================================================*/   
/* VDD range */
#define VDD_RANGE                           ((uint8)(5))  
/* resistor divider scale for DC bus voltage */
#define DC_BUS_SAMPLE_SCALE                 ((uint8)((1+19.1)/1))    
/* PMW frequency, uint: Hz */
#define PMW_FREQ                            ((uint16)(20000))    
/* PWM period */
#define PWM_PERIOD                          ((uint16)(1200))   
/* clock frequency for speed measurement */
#define SPEED_MEASURE_CLOCK_FREQ            ((uint32)(320 * 1000))                              /* 300KHz */    
/* seconds per minutes */
#define SECOND_PER_MINUTE                   ((uint8)(60))    
/* operation voltage for DC bus, uint: V */
#define NORMAL_DC_VOLTAGE                   ((uint8)(24))      
/* ADC result range */    
#define ADC_RANGE                           ((uint16)(4096))        
/*============================================================================   
 * macro for ADC sampling
 *============================================================================*/   
/* ADC channel number for DC bus voltage sample */    
#define DC_BUS_CHAN                         ((uint8)(0))
/* ADC channel number for speed reference potentiometer */     
#define SPEED_REF_CHAN                      ((uint8)(1)) 
/* define the maximum resistor value for potentiometer */    
#define SPEED_POT_VALUE                     (uint16)(10000)
/* define the accepted maximum/minimum value for the sample result of speed reference potentiometer,
   minus 100 to reduce the offset at the boundary*/    
#define MAX_SPEED_REF_ADC_VALUE             (uint16)((((SPEED_POT_VALUE + 330) * ADC_RANGE)/(SPEED_POT_VALUE + 330 + 330)) - 30)    
#define MIN_SPEED_REF_ADC_VALUE             (uint16)((((330) * ADC_RANGE)/(SPEED_POT_VALUE + 330 + 330)) + 30)    
/*============================================================================   
 * macro for speed PID control
 *============================================================================*/   
/* default value for kp */
#define KP_INITIAL_VALUE                    ((uint32)(140000))
/* default value for ki */
#define KI_INITIAL_VALUE                    ((uint32)(8000))        
/* maximum duty cycle for driving PWM */    
#define MAX_PWM_DUTY                        ((uint16)(PWM_PERIOD * 1000 / 1000))                 /* 100% */
/* minimum duty cycle for driving PWM */    
#define MIN_PWM_DUTY                        ((uint16)(PWM_PERIOD * 90 / 1000))                   /* 9% */
/* speed threshold to change the Kp and Ki */
#define SWITCH_SPEED_RPM                    ((uint16)(2000))                                    /* RPM uint */ 
/* period time for executing speed PID control */
#define SPEED_PID_EXEC_INTERVAL             ((uint16)(10 * PMW_FREQ / 1000))                    /* 10ms */
/* period time for updating speed reference */
#define SPEED_REF_UPDATE_INTERVAL             ((uint16)(10 * PMW_FREQ / 1000))                  /* 10ms */    
/*============================================================================   
 * macro for preposition
 *============================================================================*/   
/* duty cycle for preposition stage */
#define PREPOSITION_PWM_DUTY                MIN_PWM_DUTY    
/* wait time for preposition stage */
#define PREPOSITION_WAIT_TIME               ((uint16)(10000))       
/*============================================================================   
 * macro for startup
 *============================================================================*/       
/* default initial start duty cycle to make rotor rotate */
#define START_PWM_DUTY                      ((uint16)(PWM_PERIOD * 300 / 1000))                 /* 30.0% */              
/* default duty cycle for free run in open loop */
#define FREERUN_PWM_DUTY                    ((uint16)(PWM_PERIOD * 208 / 1000))                 /* 20.8% */  
/* default timeout time for every startup zero crossing checking */
#define START_ZC_CHECK_PERIOD               ((uint16)(4 * SPEED_MEASURE_CLOCK_FREQ / 1000))     /*  4ms */  
/* default wait count for staying in start stage of startup */   
#define START_STAGE_WAIT_COUNT              ((uint8)(10))    
/* default interval between two duty cycle decreasing for decreasing stage of startup */
#define DEC_STAGE_INTERVAL                  ((uint8)(1))
/* default wait count for staying in free run stage of startup */
#define FREERUN_STAGE_WAIT_COUNT            ((uint8)(10)) 
/* default interval between two duty cycle increasing for increasing stage of startup */
#define ACC_STAGE_INTERVAL                  ((uint8)(10))  
/* default execution times for acceleration stage */    
#define ACC_STAGE_EXEC_COUNT                ((uint8)(200))  
    
/* timeout period increase step for acceleration state */
#define ACC_STAGE_TIME_STEP                 ((uint8)(40))    
/* duty cycle increase step for acceleration state */    
#define ACC_STAGE_DUTY_STEP                 ((uint8)(0))     
/* zero crossing count to switch to normal run state */    
#define ZC_CHECKING_STABLE_THRESHOLD        (uint8)(3)
/* threshold to reset zero crossing counter to remove accumulate error */
#define ZC_COUNT_RESET_THRESHOLD            (uint8)(10)    
/*============================================================================   
 * macro for speed reference
 *============================================================================*/     
/* initial speed reference after motor enter into closed loop */
#define INIT_SPEED_REF_RPM                  ((uint16)(1000))                                    /* uint: RPM */  
/* maximum speed reference for motor running */    
#define MAX_SPEED_REF_RPM                   ((uint16)(4000))                                    /* uint: RPM */
/* minimum speed reference for motor running */    
#define MIN_SPEED_REF_RPM                   ((uint16)(500))                                     /* uint: RPM */
/* maximum step for speed reference update */
#define MAX_SPEED_REF_UPDATE_STEP           ((uint16)400)  
/*============================================================================   
 * macro for protection
 *============================================================================*/
/* ADC sample result for normal operation voltage of DC BUS */ 
#define DC_BUS_SAMPLE_VALUE                 ((uint16)(((ADC_RANGE * NORMAL_DC_VOLTAGE) / (DC_BUS_SAMPLE_SCALE * VDD_RANGE)) & 0x0FFF))   
/* debounce checking count threshold for voltage checking */
#define PROTECTION_DEBOUNDCE_CNT            ((uint8)(10))
    
#define  MAX_CURR_LOW                       ((uint8)50)       /*1A*/
#define  MAX_CURR_MEDIUM                    ((uint8)75)       /*1.5A*/
#define  MAX_CURR_HIGH                      ((uint8)100)      /*2A*/        
/*============================================================================   
 * macro for error detection
 *============================================================================*/
/* threshold to trigger zero crossing check failure error */
#define ZC_CHECK_ERROR_THRESHOLD            ((uint16)(80 * SPEED_MEASURE_CLOCK_FREQ / 1000))    /* 80ms */

/*============================================================================   
 * macro for misc
 *============================================================================*/    
/* wait time to start speed closed loop control */    
#define SPEED_CLOSE_LOOP_DELAY              ((uint8)(20))    
/* wait time to skip noise pulse by flywheel in zero crossing checking */
#define ZC_CHECK_SKIP_COUNT                 ((uint8)(3))
/******************************************************************************
 * Structure/Enum type declaration   
 ******************************************************************************/ 
    
/* enum type for rotation direction */
typedef enum _Direction
{
	CLOCK,                                /* clock rotation direction */
	COUNTER_CLOCK                         /* counter-clock rotation direction */
}Direction_T;

/* structure type for storing BLDC motor parameters and rotation configuration */
typedef struct _BLDC_Config
{   
    /*-----------------------------------------------------------------------------*
     * motor parameters                                                            *
     *-----------------------------------------------------------------------------*/
    uint8   polePairNumber;              /* motor pole-pair number */ 
    
    /*-----------------------------------------------------------------------------*
     * general parameters                                                          *
     *-----------------------------------------------------------------------------*/
    Direction_T   direction;             /* rotation direction */    
    uint16  initSpeedRefRpm;             /* initial speed reference, in RPM uint */    
   
    /*-----------------------------------------------------------------------------*
     * PID parameters                                                              *
     *-----------------------------------------------------------------------------*/
    uint32  kp;                          /* coefficient of PI partial portion */   
    uint32  ki;                          /* coefficient of PI integration portion */
    
    /*-----------------------------------------------------------------------------*
     * preposition parameters                                                      *
     *-----------------------------------------------------------------------------*/
    uint16  prepositionTime;             /* time for preposition stage */
    uint16  prepositionDuty;             /* duty cycle for preposition stage */
    
    /*-----------------------------------------------------------------------------*
     * startup parameters                                                          *
     *-----------------------------------------------------------------------------*/
    uint16  startDuty;                   /* duty cycle to start rotation from stop */
    uint16  freerunDuty;                 /* duty cycle for free run (open loop mode) */
    uint16  startCheckPeriod;            /* timeout period for every startup zero 
                                            crossing checking */
    uint8   startStageWait;              /* wait count for staying in start stage */
    uint8   decStageInterval;            /* interval between two decreasing for duty
                                            cycle in decreasing stage */
    uint8   freerunStageWait;            /* wait count for staying in free run stage */
    uint8   accStageInterval;            /* interval between two increasing for duty 
                                            cycle in increasing stage */
    uint8   accStageWait;                /* execution times for acceleration stage */
    uint8   accDutyStep;                 /* increase step for duty cycle in 
                                            acceleration stage */
    uint8   accTimeStep;                 /* decrease step for checking period in 
                                            acceleration stage */
    
    /*-----------------------------------------------------------------------------*
     * normal run parameters                                                       *
     *-----------------------------------------------------------------------------*/
    uint8   speedCloseLoopWait;          /* wait count in speed open loop mode before 
                                            switching to speed closed loop mode */
    uint8   zcCheckSkipCount;            /* the count for skipping zero crossing 
                                            checking during flywheel pulse */
}BLDC_Config_T;

/******************************************************************************
 * Global variables declaration   
 ******************************************************************************/    
    
/******************************************************************************
 * Global function declaration   
 ******************************************************************************/  

/******************************************************************************
 * End of declaration   
 ******************************************************************************/  

#endif  /* __PARAMETER_H__ */
/* [] END OF FILE */
