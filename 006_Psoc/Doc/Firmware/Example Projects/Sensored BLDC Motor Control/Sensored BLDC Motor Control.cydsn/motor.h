/*******************************************************************************
* Project Name		: Sensored BLDC Motor Control
* File Name			: motor.h
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

#ifndef _MOTOR_H_
#define _MOTOR_H_
 
#include <cytypes.h>   

/* define TRUE and FALSE if not defined previously */
#ifndef FALSE
#define FALSE           (uint8)0
#endif

#ifndef TRUE
#define TRUE            (uint8)1
#endif 


/* macro definition for rotation direction */
#define CLOCK_WISE              (uint8)0
#define COUNTER_CLOCK_WISE      (uint8)1

/* Status specific*/
#define STATUS_RUN              (uint8)0x01
#define STATUS_STOP             (uint8)(STATUS_RUN + 1)
#define STATUS_ERROR            (uint8)(STATUS_STOP + 1)

#define FREQ_CAPTURE_CLK        ((uint32)(100000))
#define MOTOR_POLE_PAIRS        ((uint32)(4))

/*Command from UI*/
typedef struct
{
    uint8  run; /* Let motor running*/
} UI_CMD;

/*Data passed between UI and MC controller*/
typedef struct
{ 
   uint8   Dir; /*Direction*/
   
   uint16  speedRpm; /* Actual motor speed*/
   uint16  speedRpmRef; /* motor speed command value*/
   
   uint16  kp;   /* Proportional coefficient of PID*/
   uint16  ki;   /* Integral coefficient of PID*/
   
   uint16  maxSpeedRpm;  /* Motor parameters*/
   uint16  minSpeedRpm;  /* Motor parameters*/ 
   uint8   polePairs;        /*POLE PAIRS*/
   uint8   maxCurr;     /*Over current threshold*/   
}UI_DATA;


/*To be defined*/
typedef enum _Error{
    no_error = 0,
    overCur,
    hallError,
    lowVolt,
    highVolt
} Error_T;

extern Error_T errorState; 
extern UI_CMD UI_Cmd;
extern UI_DATA UI_Data;
extern uint16 speedRef;
extern uint16 speedCur;

extern uint16 preSpeedCur;
extern uint16 preCntCaptur;

extern uint16 pwmCnt;
extern uint16 pwmCntLocal;
extern uint8  dutyCycle;
extern uint8 stateSys;

/* Functions */
void Init_UI_FW(void);
void Init_HW(void);
void Init_UI_HW(void);
    
#endif
