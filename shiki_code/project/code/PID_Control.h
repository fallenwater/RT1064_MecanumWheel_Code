#ifndef _PID_CONTROL_H_
#define _PID_CONTROL_H_

#include "zf_common_headfile.h"
#include "pid.h"
#include "Motor.h"
#include "MCar_Control.h"

/* ================================= 宏定义区 ================================= */
/** 控制参数宏定义 */

#define MOTOR_DEADBAND 1400

/** 运动参数宏定义 */
#define DEFAULT_SPEED 8000
#define SPEED_1M6 9000
#define SPEED_1M7 10000
#define SPEED_FASTER 12000
#define DEFAULT_TURNING_RADIUS 4.2f // 默认转弯半径

// 切换阈值 (例如 10度 以内算保持，10度以外算转向)
#define ANGLE_SWITCH_THRESHOLD 30.0f
/* ================================= 结构体定义区 ================================= */

typedef struct
{
    int Motor_Low_Low_Speed;   // 极低速
    int Motor_Low_Speed;       // 低速
    int Motor_Normal_Speed;    // 正常速度
    int Motor_Medium_Speed;    // 中等速度
    int Motor_High_Speed;      // 高速
    int Motor_High_High_Speed; // 超高速
} Motor_Speed_Selection;
extern Motor_Speed_Selection Set_Motor_Speed;

// 速度梯形规划
typedef struct
{
    float target_value;  // 最终想要达到的速度
    float current_value; // 当前规划出的速度（给PID的）
    float step;          // 加速度（每周期增加多少速度）
} Ramp_Handle_t;

// 定义位姿结构体
typedef struct
{
    float x;   // 世界坐标 X (单位：编码器脉冲累加值 或 cm)
    float y;   // 世界坐标 Y
    float yaw; // 世界角度 (单位：度)
} Robot_Pose_t;

// 全局变量声明
extern Robot_Pose_t current_pose; // 当前位姿（里程计更新）
extern Robot_Pose_t target_pose;  // 目标位姿
/* ================================= 全局变量定义区 ================================= */
/** 控制模式与状态标志 */
extern bool SPEED_ENABLE; // 速度开关标志位
extern float pit0_dt;


/** 运动控制参数 */
extern int8 rotation_direction;      // 车身旋转方向（-1：逆时针，1：顺时针）
extern int8 tangential_linear_speed; // 旋转时的切向线速度

/** 速度相关参数 */
extern int16 tracking_speed;          // 循迹速度，y方向
extern int16 last_speed;              // 循迹模式下的速度一阶滤波
extern int speed_compensation_offset; // 速度补偿偏移量

/** 电机控制参数 */
extern volatile int vel_pid_duty[4]; // 四轮占空比

extern volatile int Read_EncoderSpeed[4];        // 读取的编码器速度
extern volatile float Last_EncoderSpeed[4];      // 上一轮的编码器速度
extern volatile float Last_Last_EncoderSpeed[4]; // 上上一轮的编码器速度
extern float enc_filter_alpha_1;                 // 编码器滑动滤波 1
extern float enc_filter_alpha_2;                 // 编码器滑动滤波 2
extern volatile float Filter_EncoderSpeed[4];    // 滤波后的的编码器速度
extern volatile float Target_EncoderSpeed[4];    // 目标编码器速度
extern volatile float Filter_PPS[4];             // 滤波后的每秒脉冲当速度（PPS）
extern volatile float Target_PPS[4];             // 目标的每秒脉冲速度（PPS）

extern volatile float Read_WheelSpeed[4]; // 读取的四轮速度
extern volatile int Target_wheelspeed[4]; // 输出的实际目标轮速

extern int sum_wheelspeed[4]; // 轮速累加
extern int sum_EncoderSpeed[4];

/* ================================= 函数声明区 ================================= */
/**
 * 猛冲
 */
void rushrushrush(void);

// 功能函数
bool Check_Angle_Stable(float target_angle, float current_angle);			// 小角度稳定检查
float Calculate_Min_Angle_Error(float target_deg, float current_deg);		// 计算最小角度误差
void enc_speed_to_odom(void);



// 控制函数
void Angle_Control(void);

void Alg_Position_Control(void);
/**
 * 电机控制相关函数
 */
void Motor_Control(void);
float Get_Max_Accel(ControlMode_t mode);										  // 加速度分配策略
float Smooth_Speed_Control(float target_speed, float current_speed);              // 速度平滑控制
void PID_Param_Lookup(PID_lookup_enum pid_type, float input_val, int8 motor_idx); // 模糊PID

#endif
