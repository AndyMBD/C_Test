#include "Ifx_Math.h"
static inline Ifx_Math_Fract32 clip_q63_to_q31(Ifx_Math_Fract64 x)
{
    if (x>IFX_MATH_FRACT32_MAX)
    {
        x=IFX_MATH_FRACT32_MAX;
    }
    else if(x<IFX_MATH_FRACT32_MIN)
    {
        x=IFX_MATH_FRACT32_MIN;
    }
    
    return x;
}
