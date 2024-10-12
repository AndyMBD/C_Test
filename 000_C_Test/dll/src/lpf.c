#include "lpf.h"
stru_RC_t stru_Curr;
stru_PIRegulator_t stru_PIRegulator_Current;
int16_t lowPass_filter(stru_RC_t *rc, int16_t signal)
{
	int32_t wkg;
	wkg = (signal - (int16_t)(rc->yk_1 >> 16)) * (rc->coef);
	rc->yk_1 += wkg;
	return (rc->yk_1 >> 16);
}
/* Inc PI regulator */
int16_t PIRegulator_Inc(stru_PIRegulator_t *Reg)
{
	long ACC;
	int16_t AX;
	ACC = (long)(Reg->wInError - Reg->wLastError) * Reg->KP;
	ACC = (ACC << 4) + (long)(Reg->wInError) * Reg->KI;
	Reg->wIntegral = ACC + Reg->wIntegral;

	if (Reg->wIntegral > Reg->wUpperLimitOutput)
	{
		Reg->wIntegral = Reg->wUpperLimitOutput;
	}
	else if (Reg->wIntegral < Reg->wLowerLimitOutput)
	{
		Reg->wIntegral = Reg->wLowerLimitOutput;
	}

	AX = Reg->wIntegral >> 16;

	Reg->wLastError = Reg->wInError;

	return (AX);
}
/* Position PI regulator */
int16_t PIRegulator_Pos(stru_PIRegulator_t *Reg)
{
	long ACC;
	int16_t AX;
	ACC = (long)(Reg->wInError - Reg->wLastError) * Reg->KP;
	ACC = (ACC << 4) + (long)(Reg->wInError) * Reg->KI;
	Reg->wIntegral = ACC + Reg->wIntegral;

	if (Reg->wIntegral > Reg->wUpperLimitOutput)
	{
		Reg->wIntegral = Reg->wUpperLimitOutput;
	}
	else if (Reg->wIntegral < Reg->wLowerLimitOutput)
	{
		Reg->wIntegral = Reg->wLowerLimitOutput;
	}

	AX = Reg->wIntegral >> 16;

	Reg->wLastError = Reg->wInError;

	return (AX);
}

