#include "zf_common_typedef.h"
#include "zf_common_headfile.h"
#include "vofa.h"
#include "pid.h"
#include "zf_common_interrupt.h"
#include "PID_Control.h"
#include "imu660ra.h"
#include "total_init.h"
//// 中断控制接口（你的代码中已有）
//extern void interrupt_global_disable(int);
//extern void interrupt_global_enable(int);

/* ================================= 变量定义区 ================================= */
volatile float vofa_enc_snapshot[8];				// vofa记录四个编码器读取的参数截取
volatile bool vofa_enc_ready = false;				// vofa读取编码器标志位，如果为true，则在主函数发送思路编码器当前数据

uint8 data_buffer[32];								// 无线串口接收数据缓存
uint8 data_len;										// 无线串口接收数据长度

// 当前选中的电机索引（0表示第一个电机:左前轮）
//static int vofa_to_motor_idx = 1;
int vofa_to_motor_idx = 3;
int the_test_encoder_speed = 0;						// 平时测试用的编码器速度


/* ================================= 函数定义区 ================================= */
// 无线串口初始化
void my_wireless_init(void)
{
	
	wireless_uart_send_byte('\r');
    wireless_uart_send_byte('\n');
    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // 初始化正常 输出测试信息
	
	if(wireless_uart_init())                                                    // 判断是否通过初始化
	{
		while(1)                                                                // 初始化失败就在这进入死循环
		{
			gpio_toggle_level(CoreBoard_LED);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
			system_delay_ms(2000);                                               // 短延时快速闪灯表示异常
		}
	}
}

// 无线串口回显代码（回显上位机发送的数据并打印数据长度和数据个数等参数，官方例程仅用于测试）
void wireless_reflect_display(void)
{
	data_len = wireless_uart_read_buffer(data_buffer, 32);                    // 查看是否有消息 默认缓冲区是 WIRELESS_UART_BUFFER_SIZE 总共 64 字节
	if(data_len != 0)                                                       // 收到了消息 读取函数会返回实际读取到的数据个数
	{
		wireless_uart_send_buffer(data_buffer, data_len);                     // 将收到的消息发送回去
		memset(data_buffer, 0, 32);
		func_uint_to_str((char *)data_buffer, data_len);
		wireless_uart_send_string("\r\ndata len:");                                 // 显示实际收到的数据信息
		wireless_uart_send_buffer(data_buffer, strlen((const char *)data_buffer));    // 显示收到的数据个数
		wireless_uart_send_string(".\r\n");
	}
}

/* ================================= vofa读取下位机传递的参数 ================================= */
void vofa_send_task(void)
{
    if (!vofa_enc_ready) return;					// 如果没进入中断，标志位为false，不发送
    vofa_enc_ready = false;

    float a = vofa_enc_snapshot[0];
    float b = vofa_enc_snapshot[1];
    float c = vofa_enc_snapshot[2];
    float d = vofa_enc_snapshot[3];
	float vofa_target_enc_speed  = vofa_enc_snapshot[4];
	float duty  = vofa_enc_snapshot[5];
	float out_duty_lb = vofa_enc_snapshot[6];
//	float out_duty_rb = vofa_enc_snapshot[7];
//	float t  = vofa_enc_snapshot[6];
	float vlf = sum_EncoderSpeed[0];
	float vrf = sum_EncoderSpeed[1];
	float vlb = sum_EncoderSpeed[2];
	float vrb = sum_EncoderSpeed[3];
	float ori_enc = Read_EncoderSpeed[vofa_to_motor_idx];

//	float v0, v1, v2, v3;

    char buf[128];

    // ? 带前缀 channels:
    int len = snprintf(buf, sizeof(buf),
                       "channels:%ld,%ld,%ld,%ld,%ld,%.4f,%.4f,%ld,%ld,%ld,%ld,%ld,%.4f,%.6f,%.6f,%.4f,%.4f,%.4f\n",
						(long)a, (long)b, (long)c, (long)d, 													// 四轮编码器数据
						(long)vofa_target_enc_speed, (float)duty,(float)Angle_now,(long)time_count_10ms,		// 目标速度；当前占空比；当前角度；10ms计数器
						(long)vlf, (long)vrf, (long)vlb, (long)vrb, (float)ori_enc,								// 四轮速度（总编码器数据）；实际输出占空比（无滤波）
						(float)hvel_pid[vofa_to_motor_idx].Kp,(float)hvel_pid[vofa_to_motor_idx].Ki,(float)Target_EncoderSpeed[vofa_to_motor_idx],		// 速度环Kp、Ki、目标速度Target
						(float)current_pose.x, (float)current_pose.y				// 当前位姿
//						(float)out_duty_lb,(float)out_duty_rb
);							// 真实输出的两个后轮占空比，因为左后轮电机内部齿轮卡坏了						

	
//	int len1 = snprintf(buf, sizeof(buf),
//                       ":%.3f,%.3f,%.3f,%.3f\n",
//                       v0, v1, v2, v3);
	
    // 或者：不带前缀
    // int len = snprintf(buf, sizeof(buf),
    //                    "%ld,%ld,%ld,%ld\n",
    //                    (long)a, (long)b, (long)c, (long)d);

    if (len > 0)
    {
		wireless_uart_send_string(buf);
        
    }
}



/* ================================= vofa调参发送数据给下位机 ================================= */
/*
这套代码支持以下调参方式（除去vofa自带的控件调参的方式）手动输入参数也可以：
①：
	dump

②：
	SET Kp 2 3.5

③：
	name:3
	Kp:1.2
	Ki:0.1
*/


// helper: trim leading/trailing spaces
/* trim 前后空白 */
static void trim(char *s) 
{
    char *p = s;
    while (*p && isspace((unsigned char)*p)) p++;
    if (p != s) memmove(s, p, strlen(p) + 1);
    size_t len = strlen(s);
    while (len && isspace((unsigned char)s[len - 1])) s[--len] = '\0';
}

// 处理单行命令

/* 解析并执行单行命令 */
static void process_command_line(char *line)
{
    char resp[128];
    trim(line);
    if (line[0] == '\0') return;

    int idx;
    float val;

    float val1, val2, val3;
    // name: / motor:
    if (sscanf(line, "name:%d", &idx) == 1 || sscanf(line, "motor:%d", &idx) == 1) {
        if (idx >= 0 && idx < 4) {
            vofa_to_motor_idx = idx;
            snprintf(resp, sizeof(resp), "OK: select motor %d\r\n", vofa_to_motor_idx);
            wireless_uart_send_string(resp);
        } else {
            wireless_uart_send_string("ERR: motor index out of range (0-3)\r\n");
        }
        return;
    }

	if (sscanf(line, "Motor_Select:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
        vofa_to_motor_idx = val;

        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 调参对象:%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	
    // Vel_Kp: 速度环Kp, 通过 vofa.c 中 全局变量 vofa_to_motor_idx 选择要调参的轮子
    if (sscanf(line, "Vel_Kp:%f", &val) == 1) 
	{
        uint32 prim = interrupt_global_disable();
        hvel_pid[vofa_to_motor_idx].Kp = val;
        // 推荐：改变参数时重置状态以避免突变
        hvel_pid[vofa_to_motor_idx].error_last = 0.0f;
        hvel_pid[vofa_to_motor_idx].error_now  = 0.0f;
        hvel_pid[vofa_to_motor_idx].output     = 0.0f;
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: motor%d Vel_Kp=%.4f\r\n", vofa_to_motor_idx, val);
        wireless_uart_send_string(resp);
        return;
    }

    // Vel_Ki: 速度环Ki, 通过 vofa.c 中 全局变量 vofa_to_motor_idx 选择要调参的轮子
    if (sscanf(line, "Vel_Ki:%f", &val) == 1) 
	{
        uint32 prim = interrupt_global_disable();
        hvel_pid[vofa_to_motor_idx].Ki = val;
        hvel_pid[vofa_to_motor_idx].error_last = 0.0f;
        hvel_pid[vofa_to_motor_idx].error_now  = 0.0f;
        hvel_pid[vofa_to_motor_idx].output     = 0.0f;
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: motor%d Vel_Ki=%.6f\r\n", vofa_to_motor_idx, val);
        wireless_uart_send_string(resp);
        return;
    }
	
	if (sscanf(line, "motor_reset:%d", &val) == 1) 
	{
        uint32 prim = interrupt_global_disable();
		if(val == 0)
		{
			
		}
		else if(val == 1)
		{
//			Target_EncoderSpeed[vofa_to_motor_idx] = the_test_encoder_speed;
		}
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: motor_state:%d \r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	/* === 设置四轮（可改为单轮）目标速度 ===*/
	if (sscanf(line, "target_speed:%f", &val) == 1) 
	{
        uint32 prim = interrupt_global_disable();
		
		the_test_encoder_speed = val;
		for(int i = 0; i < 4; i ++)
		{
			Target_EncoderSpeed[i]  = the_test_encoder_speed;
		}
		
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "target_speed:%.6f \r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	
    // alpha:
    if (sscanf(line, "alpha:%f", &val) == 1) 
	{
        if (val < 0.0f) val = 0.0f;
        if (val > 1.0f) val = 1.0f;
        uint32 prim = interrupt_global_disable();
        hvel_pid[vofa_to_motor_idx].filter_alpha = val;
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: motor%d alpha=%.3f\r\n", vofa_to_motor_idx, val);
        wireless_uart_send_string(resp);
        return;
    }

    // limit:
    if (sscanf(line, "limit:%f", &val) == 1) 
	{
        if (val < 0.0f) val = 0.0f;
        uint32 prim = interrupt_global_disable();
        hvel_pid[vofa_to_motor_idx].output_limit = val;
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: motor%d limit=%.1f\r\n", vofa_to_motor_idx, val);
        wireless_uart_send_string(resp);
        return;
    }

    // reset:
    if (strcasecmp(line, "reset") == 0) {
        uint32 prim = interrupt_global_disable();
        hvel_pid[vofa_to_motor_idx].error_last = 0.0f;
        hvel_pid[vofa_to_motor_idx].error_now  = 0.0f;
        hvel_pid[vofa_to_motor_idx].output     = 0.0f;
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: motor%d reset\r\n", vofa_to_motor_idx);
        wireless_uart_send_string(resp);
        return;
    }

    // dump:
    if (strcasecmp(line, "dump") == 0) {
        char out[256];
        int off = 0;
        for (int i = 0; i < 4; ++i) {
            off += snprintf(out + off, sizeof(out) - off,
                     "M%d Kp=%.4f Ki=%.6f alpha=%.3f limit=%.1f\r\n",
                     i, hvel_pid[i].Kp, hvel_pid[i].Ki, hvel_pid[i].filter_alpha, hvel_pid[i].output_limit);
            if (off >= (int)sizeof(out) - 50) break;
        }
        wireless_uart_send_string(out);
        return;
    }
	
	// Turn_Kp:
    if (sscanf(line, "Turn_Kp:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
        hturn_pid.Turn_Kp = val;
		hturn_pid.last_err = 0;

        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 转向环 Turn_Kp=%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	
	// Turn_Ki:
    if (sscanf(line, "Turn_Ki:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
        hturn_pid.Turn_Kp = val;
		hturn_pid.last_err = 0;
		
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 转向环 Turn_Ki=%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	
	// Turn_k_value:
    if (sscanf(line, "Turn_k_value:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
        hturn_pid.k_value = val;
		hturn_pid.last_err = 0;
		
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 转向环 Turn_k_value=%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	
	// Angle_Kp:
    if (sscanf(line, "Angle_Kp:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
        hangle_keep_pid.Kp = val;
		hangle_keep_pid.last_err = 0;
		hangle_keep_pid.integral = 0;
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 小角度环 Angle_Kp=%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }

	// Angle_Ki:
    if (sscanf(line, "Angle_Ki:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
        hangle_keep_pid.Ki = val;
		hangle_keep_pid.last_err = 0;
		hangle_keep_pid.integral = 0;
		
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 小角度环 Angle_Ki=%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }
	
	// Angle_Kd:
    if (sscanf(line, "Angle_Kd:%f", &val) == 1) {
        uint32 prim = interrupt_global_disable();
		
		hangle_keep_pid.Kd = val;
		hangle_keep_pid.last_err = 0;
		hangle_keep_pid.integral = 0;
		
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: 小角度环 Angle_Kd=%.6f\r\n", val);
        wireless_uart_send_string(resp);
        return;
    }

    // 在vofa.c中添加
    if (sscanf(line, "calibrate_encoders", &val) == 1) 
    {
        uint32 prim = interrupt_global_disable();
        
        // 重置编码器计数
        for(int i = 0; i < 4; i++)
        {
            // Encoder_Count[i] = 0;
            sum_EncoderSpeed[i] = 0;
        }
        
        interrupt_global_enable(prim);
        wireless_uart_send_string("OK: encoders calibrated\r\n");
        return;
    }

    /* === 设置目标位姿 === */
    if (sscanf(line, "target_pose:%f,%f,%f", &val1, &val2, &val3) == 3) 
    {
        uint32 prim = interrupt_global_disable();
        
        target_pose.x = val1;
        target_pose.y = val2;
        target_pose.yaw = val3;
        
        interrupt_global_enable(prim);
        snprintf(resp, sizeof(resp), "OK: target_pose=(%.3f,%.3f,%.3f)\r\n", val1, val2, val3);
        wireless_uart_send_string(resp);
        return;
    }
	
    // 兼容形式: SET Kp 0 8.92
    {
        char cmd[8];
        int mid;
        float v;
        if (sscanf(line, "SET %7s %d %f", cmd, &mid, &v) == 3) {
            if (mid >= 0 && mid < 4) {															// 选择调第一个轮子
                uint32 prim = interrupt_global_disable();
                if (strcasecmp(cmd, "Vel_Kp") == 0) hvel_pid[mid].Kp = v;						// 速度环Kp
                else if (strcasecmp(cmd, "Vel_Ki") == 0) hvel_pid[mid].Ki = v;					// 速度环Ki
                else if (strcasecmp(cmd, "alpha") == 0) hvel_pid[mid].filter_alpha = v;
                interrupt_global_enable(prim);
                snprintf(resp, sizeof(resp), "OK: SET %s %d=%.4f\r\n", cmd, mid, v);
                wireless_uart_send_string(resp);
                return;
            } else {
                wireless_uart_send_string("ERR: SET motor index invalid\r\n");
                return;
            }
        }
    }

    // 未识别
    wireless_uart_send_string("ERR: unknown cmd\r\n");
}

/* 解析接收缓冲（不用 strtok_r，移植性更好）
   buf 可能包含多条命令，用 '\n' 或 '\r' 或 ';' 分隔 */
void process_wireless_buffer(const uint8 *buf, uint32 len)
{
    if (len == 0) return;

    /* 复制一份到临时可写缓冲 */
    char tmp[256];
    if (len >= sizeof(tmp)) len = sizeof(tmp) - 1;
    memcpy(tmp, buf, len);
    tmp[len] = '\0';

    /* 把 ';' 替换为 '\n' 使统一处理 */
    for (uint32 i = 0; i < len; ++i) {
        if (tmp[i] == ';') tmp[i] = '\n';
    }

    /* 手工按行解析，避免依赖 strtok_r */
    char *p = tmp;
    char *line_start = p;
    while (*p) {
        if (*p == '\r' || *p == '\n') {
            *p = '\0';
            process_command_line(line_start);
            /* skip 连续的换行 */
            p++;
            while (*p == '\r' || *p == '\n') p++;
            line_start = p;
        } else {
            p++;
        }
    }
    /* 处理最后一行（若末尾没有换行） */
    if (line_start && *line_start) {
        process_command_line(line_start);
    }
}