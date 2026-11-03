#ifndef _PID_H_
#define _PID_H_

#include "zf_common_typedef.h"
#include "zf_driver_pwm.h"



// 计算表的大小，方便遍历
// #define VEL_FUZZY_PID_SHEET_SIZE     (sizeof(hvel_fuzzy_pid_sheet)   / sizeof(hvel_fuzzy_pid_sheet[0]  ))
#define VEL_FUZZY_PID_SHEET_SIZE     (sizeof(hvel_fuzzy_pid_sheet_0)   / sizeof(hvel_fuzzy_pid_sheet_0[0]  ))

#define TURN_FUZZY_PID_SHEET_SIZE    (sizeof(hturn_fuzzy_pid_sheet)  / sizeof(hturn_fuzzy_pid_sheet[0] ))
#define ANGLE_FUZZY_PID_SHEET_SIZE   (sizeof(hangle_fuzzy_pid_sheet) / sizeof(hangle_fuzzy_pid_sheet[0]))

#define INTEGRAL_ACTIVE_ZONE  3.0f			// 积分分离(标准PID角度环）
// ============================ 结构体定义区 ============================
typedef enum {
	Vel_PID_lookup   = 0,
	Turn_PID_lookup  = 1,
	Angle_PID_lookup = 2,
	
} PID_lookup_enum;							// PID类别查表，用于模糊PID查表指定修改的PID

// 定义查表节点结构体
typedef struct {
    float input_threshold;  // 输入阈值（这里用目标速度的绝对值）
    float kp;               // 对应的 Kp
    float ki;               // 对应的 Ki
    float kd;               // 对应的 Kd 
	float filter_alpha;
	float k_value;
} Fuzzy_PID_Sheet;			// 模糊PID表格结构体


/**
  * @brief 速度环专用PID结构体
  * @details 增量式PID + 误差低通滤波
  *          适用场景：Velocity函数等速度控制场景
  */
typedef struct 
{
    /* 控制参数 */
    float Kp;                   // 比例系数（原Velocity_Kp）
    float Ki;                   // 积分系数（原Velocity_Ki）
    /* 运行状态 */
    float error_last;           // 上一次滤波后误差
    float error_now;            // 当前滤波后误差
    float output;               // 累加后的输出值
    /* 配置参数 */
    float filter_alpha;         // 低通滤波系数（a越大，滤波越强）
    float output_limit;         // 输出限幅
    const Fuzzy_PID_Sheet* fuzzy_sheet;   // 指向模糊PID参数表的指针
} VelocityPID_HandleTypeDef;


/**
  * @brief 转向专用PID结构体
  * @details 支持动态P项，含Kp动态调整
  *          适用场景：Turn函数等转向控制场景
  */
typedef struct 
{
    /* 基础控制参数 */
    float Turn_Kp;              // 基础比例系数
    float Turn_Kd;              // 微分系数
    /* 动态P项配置 */
    float k_value;              // 动态项比例系数（与速度配置关联）(影响非线性增长率）
    float dynamic_term_limit;   // 动态项限幅
    /* 运行状态 */
    float last_err;             // 上一次误差
    /* 配置参数 */
    float output_limit;         // 输出限幅
} TurnPID_HandleTypeDef;


/**
  * @brief 标准PID结构体
  * @details 含P/I/D三项，支持积分限幅与输出限幅
  *          适用场景：PositionX_Second_Correct、PositionY_Second_Correct等标准PID控制
  */
typedef struct 
{
    /* 控制参数 */
    float  Kp;                   // 比例系数
    float  Ki;                   // 积分系数
    float  Kd;                   // 微分系数
    /* 运行状态 */
    float  last_err;             // 上一次误差
    float  integral;             // 积分项累加值
    /* 配置参数 */
    float  integral_limit;       // 积分限幅
    float  output_limit;         // 输出限幅
} StdPID_HandleTypeDef;


/* ----------------------全局变量声明区------------------------ */
extern const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_0[5];
extern const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_1[5];
extern const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_2[5];
extern const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_3[5];
extern const Fuzzy_PID_Sheet* hvel_fuzzy_pid_sheets[4]; 

extern const Fuzzy_PID_Sheet hturn_fuzzy_pid_sheet[5];

extern VelocityPID_HandleTypeDef hvel_pid[4];		
extern TurnPID_HandleTypeDef 	 hturn_pid;
extern StdPID_HandleTypeDef hpos_x_pid;
extern StdPID_HandleTypeDef hpos_y_pid;	
extern StdPID_HandleTypeDef hangle_keep_pid;		// 小角度保持PID					



void PID_Init(void);
int VelocityPID_Calculate(VelocityPID_HandleTypeDef *hpid, float actual_speed, float target_speed);
float turnPID_Calculate(TurnPID_HandleTypeDef *hpid, float actual, float target);
float StdPID_Calculate(StdPID_HandleTypeDef *pid, float actual, float target);




#endif
