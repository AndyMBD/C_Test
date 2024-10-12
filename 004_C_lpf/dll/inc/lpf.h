#ifndef _LPF_H_
#define _LPF_H_
#include "dlib_export.h"
#include "stdint.h"
// #define EXPORT_DLL __declspec(dllexport)
// extern "C" EXPORT_DLL uint32_t add(uint16_t a, uint16_t b); // int add(int a,int b)

// DLIB_EXPORT int add(int a, int b); // int add(int a,int b)

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


extern DLIB_EXPORT  stru_RC_t stru_Curr;
extern DLIB_EXPORT  stru_PIRegulator_t	stru_PIRegulator_Current;
extern DLIB_EXPORT  int16_t lowPass_filter(stru_RC_t *rc, int16_t signal);
extern DLIB_EXPORT  int16_t PIRegulator_Increment(stru_PIRegulator_t *Reg);
// extern DLIB_EXPORT uint32_t add(uint16_t a, uint16_t b); // int add(int a,int b)

#endif