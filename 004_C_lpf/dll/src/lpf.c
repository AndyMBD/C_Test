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

int16_t PIRegulator_Increment(stru_PIRegulator_t *Reg)
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
/* Position PID */
float PID_realize(float actual_val)
{
	/*计算目标值与实际值的误差*/
	pid.err = pid.target_val - actual_val;
	pid.integral += pid.err;

	/*PID算法实现*/
	pid.actual_val = pid.Kp * pid.err +
					 pid.Ki * pid.integral +
					 pid.Kd * (pid.err - pid.err_last);

	/*误差传递*/
	pid.err_last = pid.err;

	/*返回当前实际值*/
	return pid.actual_val;
}
/* Increase PID */
float PID_realize(float temp_val)
{
	/*计算目标值与实际值的误差*/
	pid.err = pid.target_val - temp_val;

	/*PID算法实现*/
	pid.actual_val += pid.Kp * (pid.err - pid.err_next) + pid.Ki * pid.err + pid.Kd * (pid.err - 2 * pid.err_next + pid.err_last);
	/*传递误差*/
	pid.err_last = pid.err_next;
	pid.err_next = pid.err;

	/*返回当前实际值*/
	return pid.actual_val;
}
void bldcm_pid_control(void)
{
	int32_t speed_actual = get_motor_speed(); // 电机旋转的当前速度

	if (bldcm_data.is_enable)
	{
		float cont_val = 0; // 当前控制值

		cont_val = PID_realize(speed_actual);

		if (cont_val < 0)
		{
			cont_val = -cont_val;
			bldcm_data.direction = MOTOR_REV;
		}
		else
		{
			bldcm_data.direction = MOTOR_FWD;
		}

		cont_val = (cont_val > PWM_PERIOD_COUNT) ? PWM_PERIOD_COUNT : cont_val; // 上限处理

		set_bldcm_speed(cont_val);

#ifdef PID_ASSISTANT_EN
		set_computer_value(SEND_FACT_CMD, CURVES_CH1, &speed_actual, 1); // 给通道 1 发送实际值
#else
		printf("实际值：%d, 目标值：%.0f，控制值: %.0f\n", speed_actual, get_pid_target(), cont_val);
#endif
	}
}