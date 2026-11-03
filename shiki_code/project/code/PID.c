#include "PID.h"
#include "zf_common_function.h"
#include "math.h"

/* ======== 速度环参数 ======== */
// const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet[5] = {
//     //  阈值(Target)    Kp      Ki     filter_alpha
//     {   3.0f,          150.0f,  0.5f,   0.3f },  // [极低速蠕动区] 需要极大的 Kp 克服静摩擦，积分给小防止过冲
//     {   8.0f,          25.0f,  0.7f,   0.3f },  // [低速起步区]   速度起来了一点，Kp 稍微降一点
//     {   15.0f,         10.0f,  0.3f,   0.3f },  // [中速过渡区]   摩擦力变小，Kp 减半，Ki 加大增强稳态
//     {   50.0f,         1.0f,  0.31f,   0.3f },  // [高速巡航区]   动量很大，Kp 必须小防止震荡，主要靠 Ki
//     {   9999.0f,       1.0f,  0.31f,   0.3f }   // [极速区/默认]  兜底参数
// };

// 在PID.c中添加

// 修改为四个独立的参数表
const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_0[5] = {
    //  阈值(Target)    Kp      Ki     filter_alpha
    {   6.0f,          950.0f, 3.0f,   0.3f ,},  
    {   12.0f,         18.0f,  1.0f,   0.3f },  
    {   18.0f,         10.0f,  0.3f,   0.3f },  
    {   50.0f,         5.0f,   1.2f,   0.3f },  
    {   9999.0f,       1.0f,   1.2f,   0.3f }   
};

const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_1[5] = {
    //  阈值(Target)    Kp      Ki     filter_alpha
    {   6.0f,          950.0f, 3.0f,   0.3f },  
    {   12.0f,         18.0f,  1.0f,   0.3f },  
    {   18.0f,         5.0f,   1.3f,   0.3f }, 
    {   50.0f,         5.0f,   1.2f,   0.3f },  
    {   9999.0f,       1.0f,   1.2f,   0.3f }   
};

const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_2[5] = {
    //  阈值(Target)    Kp      Ki     filter_alpha
    {   6.0f,          950.0f,  3.0f,   0.3f },  
    {   12.0f,         18.0f,  1.0f,    0.3f },  
    {   18.0f,         10.0f,  1.3f,    0.3f }, 
    {   50.0f,         5.0f,   1.2f,    0.3f },  
    {   9999.0f,       1.0f,   1.2f,    0.3f }  
};

const Fuzzy_PID_Sheet hvel_fuzzy_pid_sheet_3[5] = {
    //  阈值(Target)    Kp      Ki     filter_alpha
    {   6.0f,          950.0f, 3.0f,   0.3f },  
    {   12.0f,         18.0f,  1.2f,   0.3f }, 
    {   18.0f,         10.0f,  1.5f,   0.3f },  
    {   50.0f,         5.0f,   1.4f,   0.3f },  
    {   9999.0f,       1.0f,   1.4f,   0.3f }   // 
};


const Fuzzy_PID_Sheet* hvel_fuzzy_pid_sheets[4] = {
    hvel_fuzzy_pid_sheet_0,
    hvel_fuzzy_pid_sheet_1,
    hvel_fuzzy_pid_sheet_2,
    hvel_fuzzy_pid_sheet_3
};

// 四轮速度环PID定义
VelocityPID_HandleTypeDef hvel_pid[4] = {
//    [0] = { .Kp = 0.00f, .Ki = 0.00147f, .filter_alpha = 0.3f, .output_limit = 2000.0f },
//    [1] = { .Kp = 0.00f, .Ki = 0.00147f, .filter_alpha = 0.3f, .output_limit = 2000.0f },
//    [2] = { .Kp = 0.00f, .Ki = 0.00147f, .filter_alpha = 0.3f, .output_limit = 2000.0f },
//    [3] = { .Kp = 0.00f, .Ki = 0.00147f, .filter_alpha = 0.3f, .output_limit = 2000.0f },
	[0] = { .Kp = 950.00f, .Ki = 3.0f, .filter_alpha = 0.3f, .output_limit = 20000.0f },
	[1] = { .Kp = 950.00f, .Ki = 3.0f, .filter_alpha = 0.3f, .output_limit = 20000.0f },
	[2] = { .Kp = 950.00f, .Ki = 3.0f, .filter_alpha = 0.3f, .output_limit = 20000.0f },
	[3] = { .Kp = 950.00f, .Ki = 3.0f, .filter_alpha = 0.3f, .output_limit = 20000.0f },
};


/* ======== 转向环参数 ======== */
const Fuzzy_PID_Sheet hturn_fuzzy_pid_sheet[5] = {
    //  阈值(Target)    Kp      Ki     filter_alpha
    {   5.0f,          80.0f,  0.3f,   0.3f },  // 
    {   8.0f,          1.0f,  0.31f,   0.3f },  // 
    {   20.0f,         1.0f,  0.31f,   0.3f },  // 
    {   50.0f,         1.0f,  0.31f,   0.3f },  // 
    {   9999.0f,       1.0f,  0.31f,   0.3f }   // 
};

TurnPID_HandleTypeDef hturn_pid = { .Turn_Kp = 20.0f, .Turn_Kd = 0.1f, .k_value = 0.001f, 
									.dynamic_term_limit = 0.08f, 	  .output_limit = 150.0	};		// 输出限幅的是编码器速度值
			
						
									
/* ======== 位置环参数 ======== */
StdPID_HandleTypeDef hpos_x_pid 	 = {.Kp = 15.0f, .Ki = 0.00f, .Kd = 3.0f, .output_limit = 30.0f, .integral_limit = 500.0f };
StdPID_HandleTypeDef hpos_y_pid 	 = {.Kp = 15.0f, .Ki = 0.00f, .Kd = 3.0f, .output_limit = 30.0f, .integral_limit = 500.0f };
StdPID_HandleTypeDef hangle_keep_pid = {.Kp = 20.0f, .Ki = 0.00f, .Kd = 0.80f, .output_limit = 2000.0f, .integral_limit = 500.0f };		// 小角度保持PID					




/**
  * 函    数：VelocityPID_Calculate
  * 功    能：速度环专用PID计算实现
  * 参    数：hpid         - 速度环PID结构体指针
  * 参    数：actual_speed - 实际速度
  * 参    数：target_speed - 目标速度
  * 返 回 值：PID输出值（已限幅，int类型兼容原接口）
  * 备    注：增量式PID + 误差低通滤波
  */
int VelocityPID_Calculate(VelocityPID_HandleTypeDef *hpid, float actual_speed, float target_speed)	
{
    if (hpid == NULL) return 0;

//    int raw_err = target_speed - actual_speed;
//    hpid->error_now = hpid->error_last * hpid->filter_alpha + raw_err * (1 - hpid->filter_alpha);        // 低通滤波
//    int delta_err = hpid->error_now - hpid->error_last;                                                  // 本次误差与上次误差的差值
//    int delta_out = (int)((hpid->Kp) * delta_err + (hpid->Ki) * hpid->error_now);                        // 增量式PI

//    hpid->output += delta_out;  
//    hpid->output = func_limit(hpid->output, hpid->output_limit);
//    hpid->error_last = hpid->error_now;

	// 使用浮点以避免整数截断；误差方向为 target - actual（更直观）
    float raw_err = (float)target_speed - (float)actual_speed;

    // 低通滤波（保持原逻辑）
    float alpha = hpid->filter_alpha;
    hpid->error_now = hpid->error_last * alpha + raw_err * (1.0f - alpha);

    // 增量式 PI（全部用浮点）
    float delta_err = hpid->error_now - hpid->error_last;
    float delta_out = (hpid->Kp) * delta_err + (hpid->Ki) * hpid->error_now;

    hpid->output += delta_out;

    // 限幅：确保 func_limit 能处理 float（如原 func_limit 只接 int，请用下面的 float_func_limit）
    hpid->output = func_limit(hpid->output, hpid->output_limit);

    hpid->error_last = hpid->error_now;
//	
//	 printf("PID calc: tgt=%d act=%d err=%.3f d_err=%.3f dout=%.3f out=%.3f\n",
//            target_speed, actual_speed, hpid->error_now, delta_err, delta_out, hpid->output);

    return (int)hpid->output;
}


/**
  * 函    数：turnPID_Calculate
  * 功    能：转向专用PID计算实现
  * 参    数：hpid   - 转向PID结构体指针
  * 参    数：actual - 实际测量值
  * 参    数：target - 目标值
  * 返 回 值：PID输出值（已限幅）
  * 备    注：含动态P项，PD控制.小误差微调，大误差猛转，err = x*x
  */
float turnPID_Calculate(TurnPID_HandleTypeDef *hpid, float actual, float target) 
{
    if (hpid == NULL) return 0.0f;

    float err = target - actual;
    float dynamic_term = err * err * hpid->k_value;			//y = k * x * x
    dynamic_term = func_limit(dynamic_term, hpid->dynamic_term_limit);
    
    float current_Kp = hpid->Turn_Kp + dynamic_term;
    float derivative = err - hpid->last_err;
    float output = current_Kp * err + hpid->Turn_Kd * derivative;
    output = func_limit(output, hpid->output_limit);
    hpid->last_err = err;

    return output;
}


/**
  * 函    数：StdPID_Calculate
  * 功    能：标准PID计算实现
  * 参    数：hpid   - 标准PID结构体指针
  * 参    数：actual - 实际测量值
  * 参    数：target - 目标值
  * 返 回 值：PID输出值（已限幅）
  * 备    注：含P/I/D三项，支持积分限幅与输出限幅
  */
float StdPID_Calculate(StdPID_HandleTypeDef *hpid, float actual, float target) 
{
    if (hpid == NULL) return 0.0f;

    // 1. 计算当前误差（实际值 - 目标值，与原代码逻辑一致）
    float err = target - actual;

	
	if (fabs(err) < INTEGRAL_ACTIVE_ZONE) 
    {
        // 只有快到了，才启用积分，去推最后的 1 度
        hpid->integral += err;
        hpid->integral = func_limit(hpid->integral, hpid->integral_limit);
    } 
    else 
    {
        // 误差还很大时，不要积分，纯靠 P 和 D 跑
        hpid->integral = 0; 
    }
	
//    // 2. 积分项计算（含限幅）
//    hpid->integral += err;
//    hpid->integral = func_limit(hpid->integral, hpid->integral_limit);

	
    // 3. 微分项计算（误差变化量）
    float derivative = err - hpid->last_err;

    // 4. PID输出计算
    float output = hpid->Kp * err + 
                   hpid->Ki * hpid->integral + 
                   hpid->Kd * derivative;

    // 5. 输出限幅
    output = func_limit(output, hpid->output_limit);

    // 6. 更新上一次误差
    hpid->last_err = err;

    return output;
}


void PID_Init(void)
{
  // 为每个电机设置对应的参数表指针
    for (int i = 0; i < 4; i++)
    {
        hvel_pid[i].fuzzy_sheet = hvel_fuzzy_pid_sheets[i];
        // 其他初始化代码...
    }
	
	
	
}


//int32 func_limit(int32* integral,int32* limit_integral)