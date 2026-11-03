#ifndef _MCAR_CONTROL_H
#define _MCAR_CONTROL_H

#include "zf_common_typedef.h"
#include "PID_Control.h"

/* ============================ 宏定义区 ============================ */
// ================================================================
// 1. 物理机械参数 (基于你的实测)
// ================================================================

// 尺寸参数 (单位：米)
#define CAR_WHEEL_DIAMETER      0.063f      // 轮子直径
#define CAR_WHEEL_CIRCUM        (CAR_WHEEL_DIAMETER * 3.1415926535f) // 轮子周长

// 传动参数
// 编码器采集齿轮比：从动轮齿数 / 编码器齿数
// 轮子转一圈，编码器转 (70/30) = 2.33 圈
#define GEAR_RATIO_ENCODER      (70.0f / 30.0f)

// 编码器参数
// 注意：这里填入你单圈的有效计数值
// 根据你的实测数据推算，这里应为 1024 (如果是256线4倍频，也是填1024)
#define ENCODER_TOTAL_RESOLUTION 1024.0f


// 核心参数：1米对应的编码器脉冲总数 (实测值 12000)
// 如果发现走 1米 实际上走了 1.05米，就把这个值改大一点
#define PULSE_PER_METER     12000.0f

// 几何尺寸 (单位：米) -> 【请务必拿尺子量一下你的车轴距】
// 这里的 L 和 W 是指从“车中心”到“轮子中心”的距离
#define CAR_AXLE_L          0.19816f       // 前后轮轴距 (比如 20cm)
#define CAR_AXLE_W          0.18032f       // 左右轮轮距 (比如 18cm)

#define CAR_HALF_L          (CAR_AXLE_L / 2.0f) 
#define CAR_HALF_W          (CAR_AXLE_W / 2.0f)

// 旋转几何因子 = L + W (用于麦轮解算)
// 意味着：车身旋转 1rad/s 时，轮子需要贡献的线速度分量
#define GEOMETRY_FACTOR     (CAR_HALF_L + CAR_HALF_W)


#define CAR_GEAR_RATIO      (70.0f/14.0f) // 减速比：从动70齿 / 主动10齿(假设)
#define ENCODER_LINES       1024.0         // 编码器线数(如果是逐飞常用的)
#define ENCODER_MULTI       4           // 编码器倍频(通常是4)

/* 坐标阈值 */
// 定义位置到达阈值
#define POSITION_TOLERANCE_X 0.05f  // X方向误差阈值 (米)
#define POSITION_TOLERANCE_Y 0.05f  // Y方向误差阈值 (米)


/* ================================= 枚举定义区 ================================= */

// 四向移动方向枚举
typedef enum {
    MOVE_FORWARD = 0,
    MOVE_BACKWARD,
    MOVE_LEFT,
    MOVE_RIGHT
} Vertical_Move_Derection;

typedef enum {
    MOVE_STOP = 0,
    MOVE_VERTICAL,  // 四向移动
    MOVE_DIAGONAL,  // 斜向移动
    MOVE_ANGULAR    // 旋转移动
} MoveMode;
// 可以尝试 “外环并级（位置→速度 + 姿态→角速度）→ 内环速度环” 实现位置与角度的同时闭环控制


// 控制麦轮车运动模式：①绝对位置控制(指定地图坐标点) ②相对位置控制(上位机目标点相对于当前位置的偏移量) 
//                   ③速度控制 ④角速度控制(转向进行目标分类) ⑤停止控制
typedef enum {
	MODE_STOP = 0,                 	// 停止模式
	MODE_RACE_FULL,           		// 正式比赛模式
	MODE_DEBUG_CAM_OFFSET,    		// 调试1：接收相机位置误差.
    MODE_DEBUG_SERIAL_TARGET,		// 调试2：接收上位机XY绝对坐标
	MODE_POSE_PID_TEST,				// 位置角度PID调试
	MODE_ANGLE_PID_TEST,	 		// 测试转向pid
	MODE_DEBUG_NORMAL_VEL,	 		// 调试3：普通测试代码（直接开着速度环）
	
} ControlMode_t;

extern ControlMode_t Current_Car_Action_Mode;

// 任务状态枚举
typedef enum {
    TASK_IDLE,           		// 空闲状态
    TASK_WAIT_CMD,       		// 等待上位机指令
    TASK_MOVE_TO_POINT,  		// 移动到目标点
	TASK_ARRIVE_POINT,			// 到达过程点(路径规划分段的中继点)
    TASK_ARRIVE_DESTINATION,    // 到达目标点(箱子或终点，等待识别)
	TASK_WAIT_CLASSIFY,			// 等待识别图像
//    TASK_PUSH_BOX,       		// 推箱子
    TASK_COMPLETE,        		// 任务完成
	TASK_RETURN_START			// 返回起点		
} TaskState_t;

extern TaskState_t   Task_State;
/* ================================= 全局变量区 ================================= */
extern uint8_t Vertical_Move_Mode_Flag;                // 四向移动模式标志位
extern uint8_t Vertical_Move_Mode;                         // 四向移动模式变量
extern uint8_t Motor_Control_Mode;                 // 控制模式变量

extern float pulse_per_meter_f;     // 供其他文件调用的浮点数变量

extern TaskState_t current_task_state;
/* ================================= 函数声明区 ================================= */
void Kinematics_Init(void);
void Kinematics_Inverse_Calc(float vx, float vy, float omega);
void Odometry_Update(float dt_s);



void Task_Run(TaskState_t task_state);
void MCAR_Control(ControlMode_t Control_Mode);
void MCAR_MOVE(void);

// 任务函数
void move_to_point(void);		// 移动到目标点
void arrive_point(void);		// 
void arrive_destination(void);
void waiting_classify(void);		// 等待分类

#endif
