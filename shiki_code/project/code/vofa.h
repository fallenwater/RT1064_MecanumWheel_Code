#ifndef _VOFA_H
#define _VOFA_H

#include "zf_device_wireless_uart.h"
#include "zf_driver_delay.h"
#include "zf_common_typedef.h"
#include <stdint.h> 
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

#define CoreBoard_LED					(B9)
#define MasterBoard_Buzzle				(B11)

extern volatile float vofa_enc_snapshot[8];
extern volatile bool vofa_enc_ready;

extern uint8 data_buffer[32];								// 无线串口接收数据缓存
extern uint8 data_len;										// 无线串口接收数据长度
extern uint8 stop_vpid_flag;
extern int vofa_to_motor_idx;
extern int the_test_encoder_speed;


void my_wireless_init(void);
void vofa_send_task(void);		// 无限串口发送vofa数据获取数据


static void trim(char *s);
static void process_command_line(char *line);
void process_wireless_buffer(const uint8 *buf, uint32 len);



#endif
