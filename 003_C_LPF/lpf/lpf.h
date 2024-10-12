#ifndef _LPF_H_
#define _LPF_H_
#include "stdint.h"
typedef struct
{
    int32_t yk_1;
    uint16_t coef;
} stru_RC_t;

typedef struct 
{
    int32_t wUpperLimitOutput;       /* Lower Limit for Output limitation */
    int32_t wLowerLimitOutput;       /* Lower Limit for Output limitation */
    uint16_t KP;                      /* KP */
    uint16_t KI;                      /* KI */
	uint16_t KD;                      /* KD */
    int32_t wIntegral;               /* Integra */
    int32_t wInError;                /* Input error */
    int32_t wLastError;              /* Last Error*/
    int32_t wErrorC;
}stru_PIRegulator_t;


extern stru_RC_t stru_Curr;
extern stru_PIRegulator_t	stru_PIRegulator_Current;
extern int16_t lowPass_filter(stru_RC_t *rc, int16_t signal);
extern int16_t PIRegulator_Increment(stru_PIRegulator_t *Reg);

#endif