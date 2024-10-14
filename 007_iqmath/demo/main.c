#include <windows.h>
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#define _USE_MATH_DEFINES
#include "math.h"
#include "IQmathLib.h"
// #include "Ifx_Math_LowPass1st_F16.h"
#define DEBUG_LPF   0
#define DEBUG_PI    0
/* Don't understand _atoIQN*/
#define _atoIQN     0
#define _IQNasin_acos 1
#define _IQNatan2   0
#define _IQNdiv     0
#define _IQNexp     0
#define _IQNfrac    0
#define _IQNlog     0
#define _IQNmpy     0
#define _IQNmpyIQX  0
#define _IQNrepeat  0
#define _IQNrmpy    0
#define _IQNrsmpy   0
#define _IQNsin_cos 0
#define _IQNsqrt    0
#define _IQNtables  0
#define _IQNtables  0
#define _IQNtoa     0
#define _IQNtoF     0
#define _IQNversion 0

uint16_t data;
int16_t result;
uint16_t  count =   100;
uint16_t i;
char *Str_Tst       = "-12.3";
float Float_Tst     =   -12.3;
int32_t Str_Tst_IQ_Result;
int32_t Str_Tst_Std_Result;
float_t Flt_Tst_IQ_Result;
float_t Flt_Tst_Std_Result;
char *dup_str, *string="abcde"; 

void lpf_Test();
void pi_Test();
void _atoIQN_Test();
void _IQNasin_acos_Test();
void _IQNatan2_Test();
void _IQNdiv_Test();
void _IQNexp_Test();
void _IQNfrac_Test();
void _IQNlog_Test();
void _IQNmpy_Test();
void _IQNmpyIQX_Test();
void _IQNrepeat_Test();
void _IQNrmpy_Test();
void _IQNrsmpy_Test();
void _IQNsin_cos_Test();
void _IQNsqrt_Test();
void _IQNtables_Test();
void _IQNtables_Test();
void _IQNtoa_Test();
void _IQNtoF_Test();
void _IQNversion_Test();

void main()
{

    #if DEBUG_LPF 
        lpf_Test();
    
    #endif

    #if DEBUG_PI
        pi_Test();
    #endif

    #if _atoIQN
        _atoIQN_Test();
    #endif
   
    #if _IQNasin_acos
        _IQNasin_acos_Test();   
    #endif

    return;
}

void lpf_Test()
{
#if 0 
    for (i=0; i<count; i++) 
    {
    
        data = 100;
        result = lowPass_filter(&stru_Curr, data);
        printf("Data = %d, Result =%d\n", i,result);
    }
#endif
}

void pi_Test()
{
#if 0 
    for (i=0; i<count; i++) 
    {
        data = count;
        printf("Data = %d, Result =%d\n", data,result);
    }
#endif
}

void _atoIQN_Test()
{
    printf("**-------------------_atoIQN Test Start------------------**\n");

    printf("Float_Tst \n");
    printf("%f\n", Float_Tst);

    Str_Tst_IQ_Result=_atoIQ3(Str_Tst);
    printf("Str_Tst_IQ_Result \n");
    printf("%d\n", Str_Tst_IQ_Result);

    Str_Tst_Std_Result=Float_Tst*(8);
    printf("Str_Tst_Std_Result\n");
    printf("%d\n", Str_Tst_Std_Result);

    printf("**-------------------_atoIQN Test End---------------------**\n");
}
/* not good yet */
void _IQNasin_acos_Test()
{
    float Angle_Deg=60;// 60 deg
    float Angle_rad=Angle_Deg/180*M_PI;
    printf("**-------------------_IQNasin_acos Test Start------------------**\n");        
    printf("Result =%f\n", Angle_rad);
    printf("Result =%f\n", M_PI);

    Flt_Tst_IQ_Result=_IQ29asin(Angle_rad*pow(2, 28));
    printf("Str_Tst_IQ_Result =%d\n", Flt_Tst_IQ_Result);
    Flt_Tst_Std_Result=asin(Angle_rad);
    printf("Result =%f\n", Str_Tst_Std_Result);

    printf("**-------------------_IQNasin_acos Test End------------------**\n");  
}
void _IQNatan2_Test()
{

}
void _IQNdiv_Test()
{

}
void _IQNexp_Test()
{

}
void _IQNfrac_Test()
{

}
void _IQNlog_Test()
{

}
void _IQNmpy_Test()
{

}
void _IQNmpyIQX_Test()
{

}
void _IQNrepeat_Test()
{

}
void _IQNrmpy_Test()
{

}
void _IQNrsmpy_Test()
{

}
void _IQNsin_cos_Test()
{

}
void _IQNsqrt_Test()
{

}
void _IQNtables_Test()
{

}
void _IQNtoa_Test()
{

}
void _IQNtoF_Test()
{

}
void _IQNversion_Test()
{

}