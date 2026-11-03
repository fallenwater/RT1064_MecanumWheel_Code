#include "PID_Control.h"
#include "vofa.h"
#include "math.h"
#include "total_init.h"

// ============================ 全局变量定义区 ============================
/** 控制模式与状态标志初始化 */
bool SPEED_ENABLE = false; // 速度开关标志位
float pit0_dt = 0.003f;


// 最大加速度限制 (脉冲/周期?)
#define MAX_ACCEL 0.5f // 根据实际需求调整

/** 角度控制参数初始化 **/
#define ANGLE_DEADZONE 2.0f    // 角度死区范围（度）
#define ANGLE_STABLE_TIME 1000 // 稳定保持时间（ms）

static uint32_t angle_stable_timer = 0;
static bool angle_stable_flag = false;

/** 电机控制参数初始化 */
volatile int vel_pid_duty[4] = {0, 0, 0, 0};             // 四轮占空比
volatile int Read_EncoderSpeed[4] = {0, 0, 0, 0};        // 读取的编码器速度
volatile float Last_EncoderSpeed[4] = {0, 0, 0, 0};      // 上一轮的编码器速度
volatile float Last_Last_EncoderSpeed[4] = {0, 0, 0, 0}; // 上上一轮的编码器速度
float enc_filter_alpha_1 = 0.3;                          // 编码器滑动滤波 1
float enc_filter_alpha_2 = 0.4;                          // 编码器滑动滤波 2
volatile float Filter_EncoderSpeed[4] = {0, 0, 0, 0};    // 滤波后的的编码器速度
volatile float Target_EncoderSpeed[4] = {0, 0, 0, 0};    // 目标编码器速度
volatile float Filter_PPS[4] = {0.0f};                   // 滤波后的每秒脉冲速度（PPS）
volatile float Target_PPS[4] = {0.0f};                   // 目标的每秒脉冲速度（PPS）

volatile float Read_WheelSpeed[4] = {0, 0, 0, 0}; // 读取的四轮速度
volatile int Target_wheelspeed[4] = {0, 0, 0, 0}; // 输出的实际目标轮速
int sum_EncoderSpeed[4] = {0, 0, 0, 0};
int sum_wheelspeed[4] = {0, 0, 0, 0}; // 轮速累加
int sum_odom[4] = {0, 0, 0, 0};       // 总里程计算

/** 里程计数据 **/

Robot_Pose_t current_pose = {0, 0, 0}; // 当前位姿（里程计更新）
Robot_Pose_t target_pose = {0, 0, 0};  // 目标位姿

Motor_Speed_Selection Set_Motor_Speed = {
    .Motor_Low_Low_Speed = 10,
    .Motor_Low_Speed = 15,
    .Motor_Normal_Speed = 20,
    .Motor_Medium_Speed = 25,
    .Motor_High_Speed = 30,
    .Motor_High_High_Speed = 35
    //
    //	.Motor_Low_Low_Speed = 6000,
    //	.Motor_Low_Speed = 7000,
    //	.Motor_Normal_Speed = 8000,
    //	.Motor_Medium_Speed = 9000,
    //	.Motor_High_Speed = 10000,
    //	.Motor_High_High_Speed = 11000
};

/* ------------------- 函数定义区 -------------------- */

// 编码器轮速转换为里程
void enc_speed_to_odom(void)
{
    for (int i = 0; i < 4; i++)
    {
        sum_EncoderSpeed[i] += Read_EncoderSpeed[i]; // 累加瞬时编码器数据，用于计算总里程
        sum_odom[i] = sum_EncoderSpeed[i];           // 通过轮径解算距离，暂无相关数据
    }
}

// ================= 私有函数 =================
static float Limit_Angle_180(float angle)
{
    while (angle > 180.0f)
        angle -= 360.0f;
    while (angle < -180.0f)
        angle += 360.0f;
    return angle;
}




// 计算最小角度差 (结果范围 -180 到 180)
// 返回值 > 0 表示需要逆时针转，< 0 表示需要顺时针转
float Calculate_Min_Angle_Error(float target_deg, float current_deg)
{
    float err = target_deg - current_deg;

    // 将误差限制在 -180 ~ 180 之间
    while (err > 180.0f)
        err -= 360.0f;
    while (err < -180.0f)
        err += 360.0f;

    return err;
}


bool Check_Angle_Stable(float target_angle, float current_angle)
{
    float err = fabs(Calculate_Min_Angle_Error(target_angle, current_angle));
    
    if(err < ANGLE_DEADZONE)
    {
        if(!angle_stable_flag)
        {
            angle_stable_flag = true;
            angle_stable_timer = pit_get_millis(); // 获取当前时间
        }
        else if((pit_get_millis() - angle_stable_timer) > ANGLE_STABLE_TIME)
        {
            return true; // 角度稳定
        }
    }
    else
    {
        angle_stable_flag = false;
    }
    return false;
}

// PID 角度控制算法
void Angle_Control(void)
{
    float err_yaw_real = Calculate_Min_Angle_Error(target_pose.yaw, current_pose.yaw);
    float pid_out_val = 0.0f;
    float set_omega_rad = 0.0f;

    // 取绝对值判断误差大小
    float err_abs = (err_yaw_real >= 0) ? err_yaw_real : -err_yaw_real;

    if (err_abs < ANGLE_SWITCH_THRESHOLD)
    {
        // === 模式 A：小角度保持 (用标准 PID) ===
        pid_out_val = StdPID_Calculate(&hangle_keep_pid, 0.0f, err_yaw_real);
    }
    else
    {
        // === 模式 B：大角度转向 (用非线性 PID) ===
        pid_out_val = turnPID_Calculate(&hturn_pid, 0.0f, err_yaw_real);
    }

    // 转换单位：假设 PID 输出的是 度/秒 -> 转为 弧度/秒
    set_omega_rad = pid_out_val * (3.14159f / 180.0f);

    // 这里的正负号取决于电机安装方向和轮子滚轮方向
    // 如果发现某轮反了，直接在这里加负号
    float v_lf = -set_omega_rad * GEOMETRY_FACTOR;
    float v_rf = +set_omega_rad * GEOMETRY_FACTOR;
    float v_lb = -set_omega_rad * GEOMETRY_FACTOR;
    float v_rb = +set_omega_rad * GEOMETRY_FACTOR;

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
    // 改成计算目标速度的每秒脉冲数
    Target_EncoderSpeed[0] = (float)(enc_lf_per_sec * dt);
    Target_EncoderSpeed[1] = (float)(enc_rf_per_sec * dt);
    Target_EncoderSpeed[2] = (float)(enc_lb_per_sec * dt);
    Target_EncoderSpeed[3] = (float)(enc_rb_per_sec * dt);
}


// ================= 核心功能：位置闭环控制算法 =================
void Alg_Position_Control(void)
{
    // ---------------- 1. 角度环计算 (最优先) ----------------
    // 计算真正的最小误差 (处理了 179 -> -179 这种突变)
    float err_yaw_real = Calculate_Min_Angle_Error(target_pose.yaw, current_pose.yaw); // 这里误差已经算过了，后续的实际值直接代入0即可
    float pid_out_val = 0.0f;
	float setpoint_omega_rad = 0.0f;
	
    // --- 策略切换逻辑 ---
    // 取绝对值判断误差大小
    float err_abs = (err_yaw_real >= 0) ? err_yaw_real : -err_yaw_real;

    if (err_abs < ANGLE_SWITCH_THRESHOLD)
    {
        // === 模式 A：小角度保持 (用标准 PID) ===
        pid_out_val = StdPID_Calculate(&hangle_keep_pid, 0.0f, err_yaw_real);
    }
    else
    {
        // === 模式 B：大角度转向 (用非线性 PID) ===
        pid_out_val = turnPID_Calculate(&hturn_pid, 0.0f, err_yaw_real);
    }

    // 转换单位：假设 PID 输出的是 度/秒 -> 转为 弧度/秒
    setpoint_omega_rad = pid_out_val * (3.14159f / 180.0f);

    // ---------------- 2. 位置环计算 ----------------
    // 计算世界坐标系误差
    float err_x_world = target_pose.x - current_pose.x;
    float err_y_world = target_pose.y - current_pose.y;

    // 坐标变换 (World -> Robot)
    float rad = current_pose.yaw * (3.1415926f / 180.0f);
    float cos_a = cosf(rad);
    float sin_a = sinf(rad);

    float err_x_robot = err_x_world * cos_a + err_y_world * sin_a;
    float err_y_robot = -err_x_world * sin_a + err_y_world * cos_a;

    // 外环 PID 计算
    float setpoint_vx = StdPID_Calculate(&hpos_x_pid, 0, err_x_robot);
    float setpoint_vy = StdPID_Calculate(&hpos_y_pid, 0, err_y_robot);

    // ---------------- 3. 融合输出 ----------------
    Kinematics_Inverse_Calc(setpoint_vx, setpoint_vy, setpoint_omega_rad);
}

// void angle_control(void)
// {
// 	Kinematics_Inverse_Calc(0.0f, 0, 0);
// }

void Motor_Control(void)
{
	// --- 第一步：根据模式计算外环，得出四个轮子的 Target_EncoderSpeed ---
	MCAR_Control(Current_Car_Action_Mode);				// 输入当前车俩行动模式
//    SPEED_ENABLE = true;
    // --- 第二步：内环速度环（永远执行，除非硬件使能关掉） ---
    if (SPEED_ENABLE == true)
    {
        float out_duty[4] = {0};
        for (int i = 0; i < 4; i++)
        {
            // 调试用指定目标参数
            //			vel_pid_duty[i] = VelocityPID_Calculate(&hvel_pid[i], Read_EncoderSpeed[i], Target_EncoderSpeed[i]);

            // 应用平滑加减速控制
            float smooth_target = Smooth_Speed_Control(Target_EncoderSpeed[i], Filter_EncoderSpeed[i]);
            
			// 将实际编码数据转换回滤波后的脉冲/s并传入PID
//            PID_Param_Lookup(Vel_PID_lookup, smooth_target, i); // 速度环模糊PID，传入当前速度进行计算
//            PID_Param_Lookup(Vel_PID_lookup, Target_EncoderSpeed[i], i); // 传入实际目标速度才能根据速度调整KpKi

			
            vel_pid_duty[i] = VelocityPID_Calculate(&hvel_pid[i], Filter_EncoderSpeed[i], Target_EncoderSpeed[i]);
            // vel_pid_duty[i] = VelocityPID_Calculate(&hvel_pid[i], Filter_EncoderSpeed[i], smooth_target);

            if (fabs(Target_EncoderSpeed[i]) > 0.01f)
            {
                // 检测方向突变并计算额外补偿
                bool direction_change = (Filter_EncoderSpeed[i] * Target_EncoderSpeed[i] < 0);
				float deadband_comp = 0;
				if (i == 2)  { deadband_comp = (Target_EncoderSpeed[i] > 0) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;}		// 左后轮电机内部有问题，启动难
//				else if (i == 3) { deadband_comp = (Target_EncoderSpeed[i] > 0) ? MOTOR_DEADBAND + 200 : -MOTOR_DEADBAND - 200;}
				else		 { deadband_comp = (Target_EncoderSpeed[i] > 0) ? MOTOR_DEADBAND : -MOTOR_DEADBAND;}
					
                // 计算基础死区补偿（考虑方向）
                
				
                float extra_comp = (direction_change && fabs(Filter_EncoderSpeed[i]) > 5.0f) ?      // 小速度不补偿，避免震荡；大速度突变才补偿
                                (Target_EncoderSpeed[i] > 0 ? 200.0f : -200.0f) : 0.0f;
                
                // 组合输出
                out_duty[i] = vel_pid_duty[i] + deadband_comp + extra_comp;     // PID 输出 + 死区补偿 + 方向突变补偿
            }
            else // target_speed == 0
            {
                out_duty[i] = 0;
                hvel_pid[i].output = 0;
                hvel_pid[i].error_last = 0;
                hvel_pid[i].error_now = 0;
            }
        }

        // 四轮占空比设置
		Motor_Set_Duty(MOTOR_NUM_1, out_duty[0]);
        Motor_Set_Duty(MOTOR_NUM_2, out_duty[1]);
        Motor_Set_Duty(MOTOR_NUM_3, out_duty[2]);
        Motor_Set_Duty(MOTOR_NUM_4, out_duty[3]);
//		
//		vofa_enc_snapshot[6] = out_duty[2];
//		vofa_enc_snapshot[7] = out_duty[3];
		
//		Motor_Set_Duty(MOTOR_NUM_1, 4000);
//        Motor_Set_Duty(MOTOR_NUM_2, 4000);
//        Motor_Set_Duty(MOTOR_NUM_3, 4000);
//        Motor_Set_Duty(MOTOR_NUM_4, 4000);
		
//        Motor_Set_Duty(MOTOR_NUM_1, vel_pid_duty[0]);
//        Motor_Set_Duty(MOTOR_NUM_2, vel_pid_duty[1]);
//        Motor_Set_Duty(MOTOR_NUM_3, vel_pid_duty[2]);
//        Motor_Set_Duty(MOTOR_NUM_4, vel_pid_duty[3]);
    }
    else
    {
        for (int i = 0; i < 4; i++)
            Motor_Set_Duty(i, 0);
    } // 如果SPEED_ENABLE为0，强制停下使能
}

/**
 * 获取不同控制模式下的最大加速度值
 * @param mode 控制模式枚举类型
 * @return 返回对应模式下的最大加速度值
 */
float Get_Max_Accel(ControlMode_t mode)
{
    // 使用switch语句根据不同的控制模式返回对应的加速度值
    switch (mode)
    {
		// 比赛模式，返回最大加速度0.5
		case MODE_RACE_FULL:
			return 8.0f;
		// 调试正常速度模式，返回中等加速度0.5
		case MODE_DEBUG_NORMAL_VEL:
			return 5.0f;
		// 默认情况，返回较小加速度0.3
		default:
			return 5.0f;
    }
}

float Smooth_Speed_Control(float target_speed, float current_speed)
{
    float speed_diff = target_speed - current_speed;

    // // 检测方向突变：当前速度与目标速度符号相反且差值较大
    // if ((current_speed * target_speed < 0) && (fabs(speed_diff) > 5.0f))
    // {
    //     // 方向突变时，直接返回目标速度，不进行平滑限制
    //     return target_speed;
    // }

    float position_error = sqrtf((target_pose.x - current_pose.x)*(target_pose.x - current_pose.x)
                                +(target_pose.y - current_pose.y)*(target_pose.y - current_pose.y));    // 欧式距离
    
	float max_change = Get_Max_Accel(Current_Car_Action_Mode);		// 不同控制模式下采用不同的加速度策略
    // 根据误差动态调整加速度
    // 使用平滑过渡而非突变
    float accel_factor = 1.0f;
    if (fabs(position_error) > 0.8) {
        accel_factor = 1.2f;
    } else if (fabs(position_error) < 0.2) {
        accel_factor = 0.8f;
    }
    // 使用低通滤波平滑加速度因子
    static float last_accel_factor = 1.0f;
    accel_factor = 0.9f * last_accel_factor + 0.1f * accel_factor;
    last_accel_factor = accel_factor;
    max_change *= accel_factor;

    // 方向突变处理：先减速到0再反向加速
    if ((current_speed * target_speed < 0) && (fabs(speed_diff) > 5.0f))
    {
        if (fabs(current_speed) > max_change) {
            return current_speed + (current_speed > 0 ? -max_change : max_change);
        } else {
            return (target_speed > 0 ? max_change : -max_change);
        }
    }
    
    // 正常速度变化限制
    if (fabs(speed_diff) > max_change)
    {
        return current_speed + (speed_diff > 0 ? max_change : -max_change);
    }
    else
    {
        return target_speed;
    }
}

/**
 * @brief 查表获取 PID 参数
 * @param input_val  查表的依据（这里建议传 target_speed 的绝对值）
 * @param pid_handle PID 句柄指针，函数会直接修改里面的 Kp, Ki, Kd 登参数
 */
void PID_Param_Lookup(PID_lookup_enum pid_type, float input_val, int8 motor_idx)
{
    // 取绝对值，只看速度大小，不看方向
    float val_abs = fabs(input_val);

    switch (pid_type)
    {
    case Vel_PID_lookup:
        // 遍历表格

        // 遍历对应电机的参数表
        for (int i = 0; i < VEL_FUZZY_PID_SHEET_SIZE; i++)
        {
            // 找到了第一个"大于当前速度"的档位
            if (val_abs <= hvel_fuzzy_pid_sheets[motor_idx][i].input_threshold)
            {
                // 直接赋值
                hvel_pid[motor_idx].Kp = hvel_fuzzy_pid_sheets[motor_idx][i].kp;
                hvel_pid[motor_idx].Ki = hvel_fuzzy_pid_sheets[motor_idx][i].ki;
                hvel_pid[motor_idx].filter_alpha = hvel_fuzzy_pid_sheets[motor_idx][i].filter_alpha;
                return; // 找到就退出
            }
        }
        break;
    case Turn_PID_lookup:
        // 遍历表格
        for (int i = 0; i < TURN_FUZZY_PID_SHEET_SIZE; i++)
        {
            // 找到了第一个“大于当前速度”的档位
            if (val_abs <= hturn_fuzzy_pid_sheet[i].input_threshold)
            {
                // 直接赋值
                hturn_pid.Turn_Kp = hturn_fuzzy_pid_sheet[i].kp;
                hturn_pid.Turn_Kd = hturn_fuzzy_pid_sheet[i].kd;
                hturn_pid.k_value = hturn_fuzzy_pid_sheet[i].k_value;
                return; // 找到就退出
            }
        }
        break;
    case Angle_PID_lookup:
        //			// 遍历表格
        //			for (int i = 0; i < ANGLE_FUZZY_PID_SHEET_SIZE; i++)
        //			{
        //				// 找到了第一个“大于当前速度”的档位
        //				if (val_abs <= hangle_fuzzy_pid_sheet[i].input_threshold)
        //				{
        //					// 直接赋值
        //					hturn_pid.Turn_Kp = hturn_fuzzy_pid_sheet[i].kp;
        //					hturn_pid.Turn_Kd = hturn_fuzzy_pid_sheet[i].kd;
        //					hturn_pid.k_value = hturn_fuzzy_pid_sheet[i].k_value;
        //					return; // 找到就退出
        //				}
        //			}
        break;
    }
}
