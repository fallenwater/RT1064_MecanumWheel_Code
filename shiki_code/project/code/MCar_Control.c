#include "zf_common_typedef.h"
#include "math.h"
#include "PID.h"
#include "vofa.h"
#include "imu660ra.h"
#include "PID_Control.h"
#include "MCar_Control.h"

// 这里的正负号需要根据实际电机接线调整
// 1. 先把车架起来
// 2. 给 forward 速度，看四个轮子是不是都往前转。如果有反的，去底层改电机方向或者在这里改符号
// 3. 给 rotate 速度，看是不是左边往后，右边往前。
#define WHEEL_DIR_LF  1
#define WHEEL_DIR_RF  1
#define WHEEL_DIR_LB  1
#define WHEEL_DIR_RB  1

/* ================================= 变量定义区 ================================= */
// 定义默认的电机控制模式状态
ControlMode_t Current_Car_Action_Mode = MODE_POSE_PID_TEST;		        // 位姿PID调试模式	
// ControlMode_t Current_Car_Action_Mode = MODE_ANGLE_PID_TEST;			// 角度PID调试模式
//ControlMode_t Current_Car_Action_Mode = MODE_DEBUG_NORMAL_VEL;	    // 普通速度环调试模式
// 任务状态：默认位等待命令		
TaskState_t   Task_State 		 	  = TASK_WAIT_CMD;					

// 电机控制模式标志位
uint8_t Vertical_Move_Mode_Flag = 0;                // 四向移动模式标志位
uint8_t Vertical_Move_Mode;                         // 四向移动模式变量
uint8_t Motor_Control_Mode;                         // 控制模式变量


// 计算每米对应的脉冲数
float pulse_per_meter_f = 0.0f;                         // 脉冲数/米


// 机器人三轴速度（tx,ty,omega）变量定义
float robot_speed[3] = {0,0,0};
// 电机应达到的目标速度（一个PID控制周期内，电机编码器计数值的变化）

// 全局任务状态变量
TaskState_t current_task_state = TASK_IDLE;
Robot_Pose_t target_error = {0, 0, 0};  // 上位机发送的误差数据
bool task_complete_flag = false;        // 任务完成标志
//bool wait
/* ================================= 函数定义区 ================================= */

// ================= 私有函数：辅助数学 =================
//// 角度限制在 -180 ~ 180
//static float Limit_Angle_180(float angle) {
//    while(angle > 180.0f) angle -= 360.0f;
//    while(angle < -180.0f) angle += 360.0f;
//    return angle;
//}

/**
  * @函数作用：运动学参数初始化
  * @注意：在 shiki_init() 中调用
  */
void Kinematics_Init(void)
{
    // 1. 计算轮子转动一圈，编码器产生的总脉冲数 (理论值)
    // 公式：编码器单圈分辨率 * (从动齿轮 / 编码器齿轮)
    // 结果应为 1024 * (70/30) ≈ 2389.33
    float pulses_per_wheel_rev = ENCODER_TOTAL_RESOLUTION * GEAR_RATIO_ENCODER;
    
    // 2. 计算每米对应的脉冲数 (核心参数)
    // 公式：轮子单圈脉冲 / 轮子周长
    // 结果应为 2389.33 / 0.1947 ≈ 12266.88
    pulse_per_meter_f = pulses_per_wheel_rev / CAR_WHEEL_CIRCUM;
}

/**
  * @函数作用：逆运动学解算 (车身速度 -> 轮子编码器目标值)
  * @输入：
  * vx: 前进速度 (m/s)
  * vy: 横移速度 (m/s)
  * omega: 旋转角速度 (rad/s)
  * @输出：
  * 更新 Target_EncoderSpeed 全局数组 (单位：脉冲/控制周期)
  */
void Kinematics_Inverse_Calc(float vx, float vy, float omega)
{
    // 1. 麦轮逆解算公式 (计算轮子的线速度 m/s)
    // O-长方形布局标准公式：
    // V_lf = vy + vx - omega * (L+W)
    // V_rf = vy - vx + omega * (L+W)
    // V_lb = vy - vx - omega * (L+W)
    // V_rb = vy + vx + omega * (L+W)
    
    // 这里的正负号取决于电机安装方向和轮子滚轮方向
    // 如果发现某轮反了，直接在这里加负号
    float v_lf = vy + vx - omega * GEOMETRY_FACTOR; 
    float v_rf = vy - vx + omega * GEOMETRY_FACTOR;
    float v_lb = vy - vx - omega * GEOMETRY_FACTOR;
    float v_rb = vy + vx + omega * GEOMETRY_FACTOR;

    // 2. 物理转换：线速度 (m/s) -> 编码器速度 (脉冲/秒)
    // 公式：脉冲/秒 = 米/秒 * 脉冲/米
    float enc_lf_per_sec = v_lf * pulse_per_meter_f;
    float enc_rf_per_sec = v_rf * pulse_per_meter_f;
    float enc_lb_per_sec = v_lb * pulse_per_meter_f;
    float enc_rb_per_sec = v_rb * pulse_per_meter_f;

    // 3. 时间离散化：(脉冲/秒) -> (脉冲/PID周期)
    // 假设你的控制周期是 1ms (0.001s)
    // 必须与 pit_handler_0 的时间一致！
    float dt = pit0_dt; 
    
    Target_EncoderSpeed[0] = (float)(enc_lf_per_sec * dt) * WHEEL_DIR_LF;
    Target_EncoderSpeed[1] = (float)(enc_rf_per_sec * dt) * WHEEL_DIR_RF;
    Target_EncoderSpeed[2] = (float)(enc_lb_per_sec * dt) * WHEEL_DIR_LB;
    Target_EncoderSpeed[3] = (float)(enc_rb_per_sec * dt) * WHEEL_DIR_RB;
}


/**
  * @函数作用：里程计正解算 (轮子速度 -> 车身位置)
  * @输入：dt_s (两次调用的时间间隔，单位秒，如 0.001f)
  * @注意：需要在 PIT 中断里调用
  */
void Odometry_Update(float dt_s)
{
    // 1. 读取当前编码器速度 (单位: 脉冲/周期) -> 还原为 轮线速度 (m/s)
    // 轮速 = (脉冲 / dt) / (脉冲/米)
    float v_lf = (float)Read_EncoderSpeed[0] / dt_s / pulse_per_meter_f;
    float v_rf = (float)Read_EncoderSpeed[1] / dt_s / pulse_per_meter_f;
    float v_lb = (float)Read_EncoderSpeed[2] / dt_s / pulse_per_meter_f;
    float v_rb = (float)Read_EncoderSpeed[3] / dt_s / pulse_per_meter_f;

    // 2. 麦轮正解算：轮速 -> 车身速度 (m/s)
    // Vx = (LF - RF - LB + RB) / 4
    // Vy = (LF + RF + LB + RB) / 4
    float vx_robot = ( v_lf - v_rf - v_lb + v_rb) / 4.0f;
    // float vx_robot = ( v_lf + v_rf - v_lb - v_rb) / 4.0f;
    float vy_robot = ( v_lf + v_rf + v_lb + v_rb) / 4.0f;
    
    // 3. 坐标系转换 (Robot -> World) 并积分
    // 角度直接使用陀螺仪数据 (Gyro_Angle.Zdata 是角度，需转弧度)
    float rad = Gyro_Angle.Zdata * (3.1415926f / 180.0f);
    float cos_a = cosf(rad);
    float sin_a = sinf(rad);
    
    // 旋转矩阵：把车头速度投影到世界坐标系
    float vx_world = vx_robot * cos_a - vy_robot * sin_a;
    float vy_world = vx_robot * sin_a + vy_robot * cos_a;

    // 4. 累加位置 (积分)
    current_pose.x += vx_world * dt_s;
    current_pose.y += vy_world * dt_s;
    current_pose.yaw = Gyro_Angle.Zdata; 
}



/**
 * @brief 检查是否到达目标位置
 * @details 通过比较当前坐标与目标坐标的差值是否在允许的误差范围内，判断是否到达目标位置
 * @return bool 返回是否到达目标位置，true表示已到达，false表示未到达
 */
bool Check_Position_Arrived(void)
{
    // 计算当前x坐标与目标x坐标的绝对差值
    float dx = fabs(target_pose.x - current_pose.x);
    // 计算当前y坐标与目标y坐标的绝对差值
    float dy = fabs(target_pose.y - current_pose.y);
    
    // 判断x方向和y方向的差值是否都在各自的误差允许范围内
    return (dx < POSITION_TOLERANCE_X) && (dy < POSITION_TOLERANCE_Y);
}



// 麦轮完整控制函数
void MCAR_Control(ControlMode_t Control_Mode)
{	
	// --- 第一步：根据模式计算外环，得出四个轮子的 Target_EncoderSpeed ---
    switch(Control_Mode)
	{
		case MODE_DEBUG_SERIAL_TARGET:
			// 逻辑：计算当前里程 sum_odom 与 上位机目标 (target_x, target_y) 的 PID
			// 输出到 Target_EncoderSpeed[0..3]
			//            Calc_Position_To_Speed_Control();
			
			// 串口目标控制模式，执行任务状态机
	//        Task_State_Machine();
			break;

		case MODE_DEBUG_CAM_OFFSET:
			// 逻辑：直接将相机传回的误差像素值/距离值代入位置 PID
			// 输出到 Target_EncoderSpeed[0..3]
			//            Calc_Camera_Offset_Control();
			break;

		// 完整赛题任务模式
		case MODE_RACE_FULL:
			Task_Run(Task_State);
			break;
		
		// 位姿调试模式(固定位置与角度)
		case MODE_POSE_PID_TEST:
			Alg_Position_Control();					// 全程都可以采用这个来控制，只需要输入目标坐标和角度即可
			break;
		
		// 纯角度调试模式
		case MODE_ANGLE_PID_TEST:
			Angle_Control();
			break;
		
		// 速度调试模式(配对VOFA)
		case MODE_DEBUG_NORMAL_VEL:
			//			Kinematics_Inverse_Calc(0.0f, 0, 0);
			//			Alg_Position_Control();
			for(int i = 0; i < 4; i ++)
			{
				Target_EncoderSpeed[i] = the_test_encoder_speed;
//				Target_PPS[i] = the_test_encoder_speed;
			}
			break;
		
		// 强制暂停控制
		case MODE_STOP:
			break;
		
		default:
			//            for(int i=0; i<4; i++) Target_EncoderSpeed[i] = 0;
			break;
    }
}




void Task_Run(TaskState_t state)
{
	// 执行比赛逻辑（如：视觉循迹 + 避障）
	// 1. 先调整角度对准箱子
	switch (state)
	{
		case TASK_IDLE:											break;			
		case TASK_WAIT_CMD:										break;			// 等待上位机指令
		case TASK_MOVE_TO_POINT:		move_to_point();		break;			// 移动到目标点
		case TASK_ARRIVE_POINT:			arrive_point();			break;
		case TASK_ARRIVE_DESTINATION:	arrive_destination();	break;
		case TASK_WAIT_CLASSIFY:		waiting_classify();		break;
		case TASK_COMPLETE:										break;			// 任务完成
		case TASK_RETURN_START:									break;
		default:												break;	
	}
}

void move_to_point(void)
{
	if(!Check_Angle_Stable(target_pose.yaw, current_pose.yaw))
	{
		Angle_Control();						// 如果角度不平稳，就进行纯角度控制
	}
	else
	{
		/* ===
			接收到串口数据后即可更新目标位姿，进入该函数自动计算误差并进行位置环PID运算输出速度
		=== */
//		target_pose.x = uart_x;
//		target_pose.y = uart_y;
//		target_pose.yaw = uart_yaw;
		Alg_Position_Control();					// 全程都可以采用这个来控制，只需要输入目标坐标和角度即可
//		Kinematics_Inverse_Calc(10, 20, 10);		// 参数：VX,VY,OMEGA
	}
	
	// 达到目标点10误差小于一定阈值10次后，停止运动，发送数据给上位机等待分类或下一步动作 并发布标志位
	// 还没编写（
}
	
void arrive_point(void)
{
	
}

void arrive_destination(void)
{
	
}

void waiting_classify(void)
{
	for(int8 i = 0; i < 4; i++)
	{
		Target_EncoderSpeed[i] = 0;
	}

}
// 控制麦轮移动，控制逻辑为目标点与小车中心点相对距离，从而判断移动方向，并根据判断条件决定是斜向移动或者分段X->Y or Y->X移动
// 麦轮移动函数（决定移动方向）
//void MCAR_MOVE(void)
//{
//    // 根据目标点与小车中心点相对距离，判断移动方向，并根据判断条件决定是斜向移动或者分段X->Y or Y->X移动
//    
//    
//}

