#ifndef _PWM_H_
#define _PWM_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"

#define CHANNEL_NUMBER          (8)


// 前轮双电机驱动PWM
#define PWM_CH1                 (PWM2_MODULE3_CHA_D2)		// 左前
#define PWM_CH2                 (PWM2_MODULE3_CHB_D3)
#define PWM_CH3                 (PWM2_MODULE2_CHA_C10)		// 右前
#define PWM_CH4                 (PWM2_MODULE2_CHB_C11)


// 后轮双电机驱动PWM
#define PWM_CH5                 (PWM2_MODULE0_CHA_C6)		// 左后
#define PWM_CH6                 (PWM2_MODULE0_CHB_C7)
#define PWM_CH7                 (PWM2_MODULE1_CHA_C8)		// 右后
#define PWM_CH8                 (PWM2_MODULE1_CHB_C9)

void shiki_pwm_init(pwm_channel_enum pwm_channel, const uint32 pwm_freq, const uint32 pwm_init_duty);



#endif
