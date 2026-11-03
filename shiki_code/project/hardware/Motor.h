#ifndef _MOTOR_H_
#define _MOTOR_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"
#include "PWM.h"

#define CHANNEL_NUMBER          (8)
#define MOTOR_PWM_INIT(pin)		pwm_init ((pin), MOTOR_PWM_FREQ, MOTOR_INIT_DUTY)	// PWM初始化后俩参数锁定的宏函数
#define MOTOR_DIR_INIT(pin)		gpio_init((pin), GPO, GPIO_HIGH, GPO_PUSH_PULL)
//#define MOTOR_DIR_INIT(pin)		pwm_init ((pin), MOTOR_PWM_FREQ, MOTOR_INIT_DUTY)

// 左前轮
#define FL_MOTOR_PWM			(PWM_CH1)			// D12
#define FL_MOTOR_DIR			(D3)			// D14
//#define FL_MOTOR_DIR			(PWM_CH2)			// D14

// 右前轮
#define FR_MOTOR_PWM			(PWM_CH3)			// D13
#define FR_MOTOR_DIR			(C11)			// D15
//#define FR_MOTOR_DIR			(PWM_CH4)			// D15


// 左后轮
#define BL_MOTOR_PWM			(PWM_CH7)			// D0
#define BL_MOTOR_DIR			(C9)				// D2
//#define BL_MOTOR_DIR			(PWM_CH6)			// D2

// 右后轮
#define BR_MOTOR_PWM			(PWM_CH5)			// D1
#define BR_MOTOR_DIR			(C7)			// D3
//#define BR_MOTOR_DIR			(PWM_CH8)			// D3



// 电机通用参数
#define MOTOR_PWM_FREQ    	17000UL  // PWM频率（17kHz）
#define MOTOR_INIT_DUTY   	0UL      // 初始占空比（0）
#define MOTOR_MAX_DUTY    	5000     // 最大占空比限制（防止过流）
#define MOTOR_CONTROL_MAX  	2000  	 // 理想控制范围上限
/**
 * @brief 电机编号枚举
 */
typedef enum {
    MOTOR_NUM_1 = 0,  // 电机1
    MOTOR_NUM_2,      // 电机2
    MOTOR_NUM_3,      // 电机3
	MOTOR_NUM_4,      // 电机4
    MOTOR_COUNT       // 电机总数（自动计算，用于参数校验）
} MotorNum;


void Motor_Set_Duty(MotorNum motor_num, int duty);
void Motor_Init(void);

#endif
