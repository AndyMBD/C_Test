/*******************************************************************************
* Project Name		: Sensorless BLDC Motor Control
* File Name			: BLDCController.h
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
#ifndef __BLDC_CONTROLLER_H__
#define __BLDC_CONTROLLER_H__

/******************************************************************************
 * Header file including
 ******************************************************************************/ 
#include <cytypes.h>
#include "Parameters.h"
#include "Debug.h"    

/******************************************************************************
 * Macro declaration   
 ******************************************************************************/    
#ifndef TRUE
#define TRUE                                ((uint8)(0x01u))
#endif  

#ifndef FALSE
#define FALSE                               ((uint8)(0x00u))
#endif  

/******************************************************************************
 * Structure/Enum type declaration   
 ******************************************************************************/   
/* enum type for error code */
typedef enum _Error_Code
{
    NO_ERROR,                               /* no error happens */
    ZC_CHECK_ERROR,                         /* zero-crossing detection failure */
    OV_ERROR,                               /* over voltage happens */
    UV_ERROR,                               /* under voltage happens */
    FREERUN_ERROR,                          /* freerun stage fails, fail to enter closed loop */ 
    ANY_ERROR,                              /* for any unknown error */
    ERROR_SIZE                              /* variable to store count of error types */
}Error_Code_T;

/* Structure type to store control variable during BLDC rotation */
typedef struct _Run_Control
{
    uint8           runFlag;                /* flag to start motor running */
    Error_Code_T    errorCode;              /* error code for motor running */
    
    uint8           sector;                 /* one of 6 sector of BLDC control */ 
    uint8           checkFallingEdge;       /* If check fall edge of BEMF */ 
	uint8           inNormalRun;            /* flag to indicate if motor works in normal run stage*/ 
    
    uint16          speedMeasuredRpm;       /* speed measured by counter, in RPM uint */
    uint16          speedRefRpm;            /* current speed reference, may be smaller than speedGivenRpm */
    uint16          speedGivenRpm;          /* given speed reference which is desired to achieve  */
    
    uint16          commutateStamp;         /* Time stamp of latest commutation event */
    uint16          zeroCrossStamp;         /* Time stamp of previous commutation event */ 
    uint16          zeroCrossPeriod;        /* Period between two commutation events */ 
	uint16          delayTime;	            /* Delay time between zero-cross and commutation */ 
    
    int32           pidOutput;              /* PID output */
    int32           pidSpeedErr;            /* speed count error in speed PID */
}BLDC_Control_T;

/* enum type for BLDC running state */
typedef enum
{
    STOPPED         = 0x01 ,                /* motor stop state */
    FREERUNNING     = 0x02 ,                /* freerun state, open loop during motor startup */
    NORMALRUN       = 0x03 ,                /* normal run state with sensorless control after BEMF is detected */
    ERRORSTOP       = 0x04 ,                /* error state if error happens in normal run  */
    FREERUNFAILED   = 0x05 ,                /* error state if freerun stage failed */
    PREPOSITION     = 0x06 ,                /* Preposition state */
    OCERROR         = 0x07 ,                /* error state if over current happens */
} Motor_Stage_T;

/******************************************************************************
 * Global variables declaration   
 ******************************************************************************/    
extern BLDC_Control_T   BLDC_Control;              /* Structure variable for control BLDC running */
extern BLDC_Config_T    BLDC_Config;               /* Structure variable to store motor parameter */
extern Motor_Stage_T   runState;                   /* enum variable to store running state */

extern uint16 dutyCycle;                           /* duty cycle value applied on driving PWM */
extern uint16 speedRefCount;                       /* given speed reference */
extern uint16 speedCurCount;                       /* measured rotation speed */
extern uint8  needRunSpeedPID;                     /* flag to sync with PWM ticker ISR to run speed PID */
extern uint8  needUpdateSpeedRef;                  /* flag to sync with PWM ticker ISR to update speed reference */
/******************************************************************************
 * Global function declaration   
 ******************************************************************************/  
extern void BLDC_ParameterInit(void);
extern void BLDC_ControllerInit(void);
extern void BLDC_Start(void);
extern void BLDC_Stop(void);
extern void BLDC_Run(void);
extern void BLDC_UpdateSpeedRef(uint16 speed);
extern void BLDC_SpeedCloseLoop(void);
/******************************************************************************
 * End of declaration   
 ******************************************************************************/  

#endif  /* __BLDC_CONTROLLER_H__ */
/* [] END OF FILE */

