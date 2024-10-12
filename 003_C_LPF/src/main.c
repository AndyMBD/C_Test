#include "stdint.h"
#include "stdio.h"
#include "lpf.h"
#include "Ifx_Math_LowPass1st_F16.h"
#include <assert.h>
#include <stdio.h>



uint16_t data;
int16_t result;
#define DEBUG_LPF 0
#define DEBUG_PI 1

void a_output(FILE *fs, uint16_t array_s);
void a_output(FILE *fs, uint16_t array_s)
{
    int i;

    assert(fs != NULL);
    fprintf(fs, "%d ", array_s);
}

void main()
{
    uint32_t  count =   10000;
    uint32_t i;
    stru_Curr.coef  =   655*2;/* 65535*0.1 */
    stru_Curr.yk_1  =   0;
    stru_PIRegulator_Current.KP                 = 6553;/* 65535*0.1 */    
    stru_PIRegulator_Current.KI                 =   0;
    stru_PIRegulator_Current.wUpperLimitOutput  =   INT32_MAX;
    stru_PIRegulator_Current.wLowerLimitOutput  =   INT32_MIN;
    FILE *out;

#if 0 
    if((out = fopen("output.txt", "w")) == NULL){
        return -1;
    }
    /* Will "print" results in a file */
    a_output(out, array, 5);

    /* Will "print" results in console */
    a_output(stdout, array, 5);
#endif

    fclose(out);
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
        stru_PIRegulator_Current.wInError=4;
        result = PIRegulator_Increment(&stru_PIRegulator_Current);;
        printf("Data = %d, Result =%d\n", i,result);
    }
#endif

    return;
}