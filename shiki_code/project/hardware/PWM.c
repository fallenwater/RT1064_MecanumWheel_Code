#include "zf_common_typedef.h"
#include "PWM.h"

int16 duty = 0;
uint8 channel_index = 0;
pwm_channel_enum channel_list[CHANNEL_NUMBER] = {PWM_CH1, PWM_CH2, PWM_CH3, PWM_CH4,
												 PWM_CH5, PWM_CH6, PWM_CH7, PWM_CH8};

												 
void shiki_pwm_init(pwm_channel_enum pwm_channel, const uint32 pwm_freq, const uint32 pwm_init_duty)
{
	pwm_init(pwm_channel, pwm_freq, pwm_init_duty);
}