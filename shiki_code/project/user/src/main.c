#include "total_headfile.h"
#include "math.h"
// 打开新的工程或者工程移动了位置务必执行以下操作
// 第一步 关闭上面所有打开的文件
// 第二步 project->clean  等待下方进度条走完


void IO_TEST();
void sin_wireless_test(void);


char dbg[32];
int8 state = 0;
int8 pit1_test_count = 0;
int8 pit2_test_count = 0;

int main(void)
{
	
    clock_init(SYSTEM_CLOCK_600M);  // 不可删除
    debug_init();                   // 调试端口初始化
	
	shiki_init();
	
	float data = 0;
	float data_y1 = 0;
	float data_y2 = 0;
	float data_y3 = 0;
	
	the_test_encoder_speed = TEST_ENCODER_SPEED;
	for(int i = 0; i < 4; i ++)
	{
		Target_EncoderSpeed[i] = 0;		// 初始化目标速度为0
	}
	
    uint8 count = 0;


	uint8 data_send_array[4];
//	uart1_init();
    while(1)
    {
		UI();
//		data_send_array[0] = 12;		data_send_array[1] = 34;
//		data_send_array[2] = 56;		data_send_array[3] = 78;
//		uart_write_buffer(UART1_INDEX, data_send_array, 4);
//			uart_write_buffer(UART1_INDEX,dbg , 1);
//		uart_send_test();
		// 此处编写需要循环执行的代码
	
		
		data_len = wireless_uart_read_buffer(data_buffer, 32);                    // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
		if (data_len != 0) 
		{
			process_wireless_buffer(data_buffer, data_len);		// 收到无线串口数据后处理数据任务
			for(int i = 0; i < 4; i ++)
			{
				// 调试纯速度环时开启，直接将测试速度赋值给目标速度
//				Target_EncoderSpeed[i] = the_test_encoder_speed;
//				Target_PPS[i] = the_test_encoder_speed;
			}
		}

		// 位姿调试模式时，保持目标位姿不变，方便调位置环和角度环的参数
//		if (Current_Car_Action_Mode == MODE_POSE_PID_TEST)
//		{
////			target_pose.x = 0;
////			target_pose.y = 0;
////			target_pose.yaw = 0;		
//		}
		
        // if(count ++ > 100)
        // {
        //     count = 0;
        //     gpio_toggle_level(CoreBoard_LED);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
        // }
        // system_delay_ms(5);
		
////		wireless_uart_send_buffer(&data_send_array[3],1);

		
//		sin_wireless_test();
		vofa_send_task();
		


    }
}




void wireless_test(void)
{
	char dbg[32];
	sprintf(dbg, "%d %d %d %d\r\n", 12,34,56,78);
	wireless_uart_send_string(dbg);
}

// 无线串口发送vofa 正弦数据测试
void sin_wireless_test(void)
{
	
	static float data = 0;
	static float data_y1 = 0;
	static float data_y2 = 0;
	static float data_y3 = 0;
	
	
	if(data>3.14)
	{
		state = 1;
	}
	else if(data<0)
	{
		state = 0;
	}
	if(state == 1)
	{
		data-=0.0314;
	}
	else
	{
		data+=0.0314;
	}
	data_y1 = sin(data);
	data_y2 = cos(data);
	system_delay_ms(5);
//		wireless_uart_send_buffer(&data_send_array[3],1);
	sprintf(dbg, "%.2f,%.2f,%.2f,%.2f\n", data,data_y1,data_y2,1.0);
	wireless_uart_send_string(dbg);
}

// 放在主循环中，用于读取调试时的输出数据
void debug_vpid_out(void)
{
	//		// 调试输出（会导致中断延长或系统卡死）
	//		printf("Motor 0: duty=%d, actual=%d, target=%d\n", 
	//			 vel_pid_duty[0], Read_WheelSpeed[0], Target_EncoderSpeed[0]);
	//		printf("Motor 1: duty=%d, actual=%d, target=%d\n", 
	//			 vel_pid_duty[1], Read_WheelSpeed[1], Target_EncoderSpeed[1]);
	//		printf("Motor 2: duty=%d, actual=%d, target=%d\n", 
	//			 vel_pid_duty[2], Read_WheelSpeed[2], Target_EncoderSpeed[2]);
	//		printf("Motor 3: duty=%d, actual=%d, target=%d\n", 
	//			 vel_pid_duty[3], Read_WheelSpeed[3], Target_EncoderSpeed[3]);
}

