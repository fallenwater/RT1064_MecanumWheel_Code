#include "total_headfile.h"
#include "total_init.h"

int timer_count = 0;
int motor_self_duty = 1000;
int timer_count_flag = 0;
// 全局或静态加一个计数
static int pit_dbg_cnt = 0;

int time_count_10ms = 0;

/** 里程计参数初始化 */
// [电机编号 0-3][历史数据 0-3]
// history[i][0] 是最旧的，history[i][3] 是最新的
static float speed_history[4][3] = {0}; 
const float filter_weights[3] = {0.3f, 0.3f, 0.4f};       // 定义权重数组权重 从 0.1 到 0.4 递增

void shiki_init(void)
{
	gpio_init(CoreBoard_LED		, GPO, 0, GPO_PUSH_PULL);				// 核心板LED
	gpio_init(MasterBoard_Buzzle, GPO, 0, GPO_PUSH_PULL);				// 蜂鸣器初始化
	Key_Switch_init(15);												// 按键初始化，扫描周期15ms

//	shiki_uart_init(UART1_INDEX, UART1_PRIORITY, 0);	// 初始化指定串口并配置接收中断优先级
//	shiki_uart_init(UART2_INDEX, UART2_PRIORITY, 0);	// 初始化指定串口并配置接收中断优先级
//	system_delay_ms(1000);           //等待主板其他外设上电完成

	
	// logo显示
	my_tft180_init(TFT180_PORTAIT, TFT180_8X16_FONT, RGB565_BLUE, RGB565_WHITE);			//tft180初始化
	tft180_show_rgb565_image(0, 10, (const uint16 *)gImage_shiki, 128, 139, 128, 139, 0);
	system_delay_ms(500);           //等待主板其他外设上电完成
	tft180_clear();
	tft180_show_rgb565_image(0, 16, (const uint16 *)gImage_xuebao, 128, 128, 128, 128, 0);
	tft180_show_chinese(8*4,  16*9,   16, (const uint8 *)font_xuebaokeji, 4, RGB565_BLUE);           // 显示汉字
	system_delay_ms(500);
	
	// 陀螺仪初始化与零漂累计（计算时勿动）
	my_imu660ra_init();	
	imu660ra_zeroBias();

	// 麦轮四电机初始化
	Motor_Init();
	SPEED_ENABLE = true;								// 使能速度控制

	Encoder_Init();										// 四路编码器初始化
	PID_Init();											// PID初始化

	
	pit_ms_init(PIT_CH0, 3);							// pit0中断初始化： 速度计算
	pit_ms_init(PIT_CH1, 5);							// pit1中断初始化： 陀螺仪解算
	pit_ms_init(PIT_CH2, 15);							// pit2中断初始化： 按键记录
	pit_ms_init(PIT_CH3, 10);							// pit2中断初始化： 计时

	interrupt_global_enable(0);							// 使能全部中断

	// 使能无线串口
	#if USING_WIRELESS
		my_wireless_init();								// 无线串口初始化
	#endif
//						gpio_set_level(B9,1);			// 核心板LED
	// 解算初始化
	Kinematics_Init();
}


// pit0定时中断： 编码器计数
void pit_handler_0(void)
{
	// 读取四轮编码器速度（左前 右前 左后 右后）(左侧轮速度已在内部函数内部取反）
    Read_EncoderSpeed[0] = Encoder_ReadSpeed(ENCODER_NUM_1);
    Read_EncoderSpeed[1] = Encoder_ReadSpeed(ENCODER_NUM_2);
    Read_EncoderSpeed[2] = Encoder_ReadSpeed(ENCODER_NUM_3);
	Read_EncoderSpeed[3] = Encoder_ReadSpeed(ENCODER_NUM_4);

	// 更新里程计 (3ms 中断)
	Odometry_Update(pit0_dt);
	// 速度转里程
	enc_speed_to_odom();
	// 编码器滤波，如果要闭位置环不能用，会影响实际数据！！
	for(int i = 0 ; i < 4; i ++)
	{
//		float instant_pps = (float)Read_EncoderSpeed[i] / pit0_dt; 		// 瞬时速度 = 脉冲 / 时间
		// === 滑动滤波逻辑 ===
		// 1. 移动历史数据 
		speed_history[i][0] = speed_history[i][1];
		speed_history[i][1] = speed_history[i][2];
		speed_history[i][2] = Read_EncoderSpeed[i];				// 最新数据入队				
		// 2. 加权求和
		float filtered_enc = 0.0f;
		for(int k=0; k<3; k++)
		{
			filtered_enc += speed_history[i][k] * filter_weights[k];
		}
		// 3. 将 filtered_pps 传给 PID
		Filter_EncoderSpeed[i] = filtered_enc; // 传给 PID
	}

	
	// 将读取到的编码器数据转成vofa数据
	vofa_enc_snapshot[0] = Filter_EncoderSpeed[0];
	vofa_enc_snapshot[1] = Filter_EncoderSpeed[1];
	vofa_enc_snapshot[2] = Filter_EncoderSpeed[2];
	vofa_enc_snapshot[3] = Filter_EncoderSpeed[3];

	vofa_enc_snapshot[4] = the_test_encoder_speed;
	// 使能vofa编码器接收标志位，给上位机发送编码器当前数据
	vofa_enc_ready = true;									// vofa读取编码器标志位，如果为true，则在主函数发送思路编码器当前数据

	// 电机PID控制函数
	Motor_Control();
	vofa_enc_snapshot[5] = vel_pid_duty[vofa_to_motor_idx];

	// 编码器数据调试
//	if (++pit_dbg_cnt >= 50) {
//        pit_dbg_cnt = 0;
//        printf("PIT ENC: %d,%d,%d,%d  PID: %d,%d,%d,%d\r\n",
//               Read_EncoderSpeed[0], Read_EncoderSpeed[1], Read_EncoderSpeed[2], Read_EncoderSpeed[3],
//               vel_pid_duty[0], vel_pid_duty[1], vel_pid_duty[2], vel_pid_duty[3]);
//    }
}



/**
  * 函    数：pit_handler_1
  * 功    能：定时器中断处理函数，实时更新陀螺仪角度
  * 参    数：无
  * 返 回 值：无
  * 备    注：1. 建议配置为5ms周期，与采样间隔保持一致；
  *           2. 角度计算公式：角度 = 角速度×时间（时间单位：秒）；
  *           3. 0.005f对应5ms中断周期，需与硬件定时器配置匹配；
  *           4. 包含零漂补偿，提高角度计算精度
  */
// pit1定时中断： 陀螺仪
void pit_handler_1(void)
{
    // 读取并转换陀螺仪数据（原始值→度/秒）
    imu660ra_get_gyro();
    tra_gyro_z = imu660ra_gyro_transition(imu660ra_gyro_z);
    
    // 陀螺仪角度积分（角度 = 角速度×时间，时间单位：秒）
    // 注：0.005f对应5ms中断周期，需与硬件定时器配置保持一致
    Gyro_Angle.Zdata += (tra_gyro_z * 0.005f - Gyro_Bias.Zdata*1.02);
    
    // 更新当前角度（优先使用融合角度，此处保留原逻辑）
    Angle_now = Gyro_Angle.Zdata;
}


// pit2定时中断： 按键扫描
void pit_handler_2(void)
{
	key_scanner();
	UI_Read_Button();

}

void pit_handler_3(void)
{
	time_count_10ms = time_count_10ms + 1;
//	vofa_enc_snapshot[6] = time_count_10ms;
}


uint32_t pit_get_millis(void)
{
	return time_count_10ms*10; // 返回毫秒
}
