#include <windows.h>
#include "stdint.h"
#include "stdio.h"
#include "lpf.h"
// #include "Ifx_Math_LowPass1st_F16.h"
uint16_t data;
int16_t result;
#define DEBUG_LPF 0
#define DEBUG_PI 1
void main()
{
    stru_Curr.coef  =   6553;/* 65535*0.1 */
    stru_Curr.yk_1  =   0;
    uint16_t  count =   100;
    uint16_t i;
    #if DEBUG_LPF 
    for (i=0; i<count; i++) 
    {
        data = 100;
        result = lowPass_filter(&stru_Curr, data);
        printf("Data = %d, Result =%d\n", i,result);
    }
    #endif
    #if DEBUG_PI
    for (i=0; i<count; i++) 
    {
        data = count;
        stru_PIRegulator_Current.wInError=1;
        result = PIRegulator_Inc(&stru_PIRegulator_Current);;
        printf("Data = %d, Result =%d\n", data,result);
    }
    #endif

    return;
}