#include "zf_common_typedef.h"
#include "PWM.h"
#include "Motor.h"
#include "zf_common_function.h"
#include "zf_driver_gpio.h"


// ============================ 静态数据定义区 ============================
/**
 * 电机硬件参数结构体数组
 * 将每个电机的PWM引脚、方向引脚与编号绑定，通过枚举索引直接访问
 */
typedef struct {
    pwm_channel_enum  pwm_pin;   // PWM输出引脚
	gpio_pin_enum     dir_pin;	 // 方向控制引脚
//    pwm_channel_enum  dir_pin;   
} MotorHardware;


// 匹配电机数组
static const MotorHardware Motor_Match[MOTOR_COUNT] = {
    {FL_MOTOR_PWM, FL_MOTOR_DIR},  // 电机1：绑定PWM和方向引脚
    {FR_MOTOR_PWM, FR_MOTOR_DIR},  // 电机2：绑定PWM和方向引脚
    {BL_MOTOR_PWM, BL_MOTOR_DIR},  // 电机3：绑定PWM和方向引脚
	{BR_MOTOR_PWM, BR_MOTOR_DIR}   // 电机4：绑定PWM和方向引脚
};


void Motor_Init(void)
{
	// 四轮PWM初始化
	MOTOR_PWM_INIT(FL_MOTOR_PWM);		
	MOTOR_PWM_INIT(FR_MOTOR_PWM);
	MOTOR_PWM_INIT(BL_MOTOR_PWM);
	MOTOR_PWM_INIT(BR_MOTOR_PWM);
	// 四轮DIR初始化（选用的同样是PWM引脚来配置）	
	MOTOR_DIR_INIT(FL_MOTOR_DIR);		
//	MOTOR_DIR_INIT(FR_MOTOR_DIR);
//	pwm_init (PWM_CH4, MOTOR_PWM_FREQ, MOTOR_INIT_DUTY);
	gpio_init(D15, GPO, GPIO_HIGH, GPO_PUSH_PULL);
	MOTOR_DIR_INIT(BL_MOTOR_DIR);
	MOTOR_DIR_INIT(BR_MOTOR_DIR);
	
	// 初始化速度，避免速度环无法启动
	Motor_Set_Duty(MOTOR_NUM_1, 0);
	Motor_Set_Duty(MOTOR_NUM_2, 0);
	Motor_Set_Duty(MOTOR_NUM_3, 0);
	Motor_Set_Duty(MOTOR_NUM_4, 0);
	
//    pwm_set_duty(Motor_Match[motor_num].pwm_pin, (uint32_t)duty);
//    pwm_set_duty(Motor_Match[motor_num].pwm_pin, (uint32_t)duty);
//    pwm_set_duty(Motor_Match[motor_num].pwm_pin, (uint32_t)duty);
//    pwm_set_duty(Motor_Match[motor_num].pwm_pin, (uint32_t)duty);

}

void Motor_Set_Duty(MotorNum motor_num, int duty)
{
    // 参数合法性校验：电机编号超出范围则退出
    if (motor_num >= MOTOR_COUNT) 
	{
        return;
    }

    // 占空比限幅：防止超过电机最大承受值
    duty = func_limit(duty, MOTOR_MAX_DUTY);

    // 方向控制：根据占空比正负设置方向引脚电平					占空比最大/最小值模拟 方向高低电平
    if (duty >= 0)	{gpio_set_level(Motor_Match[motor_num].dir_pin, GPIO_LOW );}
    else 				{gpio_set_level(Motor_Match[motor_num].dir_pin, GPIO_HIGH); duty = -duty;}
//	if (duty >= 0)	{pwm_set_duty(Motor_Match[motor_num].dir_pin, PWM_DUTY_MIN );}
//    else 			{pwm_set_duty(Motor_Match[motor_num].dir_pin, PWM_DUTY_MAX); duty = -duty;}
    // 输出PWM占空比
    pwm_set_duty(Motor_Match[motor_num].pwm_pin, (uint32_t)duty);
}
