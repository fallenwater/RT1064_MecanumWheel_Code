#ifndef _TOTAL_INIT_H_
#define _TOTAL_INIT_H_

#include "total_headfile.h"


#define USING_WIRELESS				(1)			// 如果没有接无限串口需要设置为0

#define TEST_ENCODER_SPEED			(0)		// 平时测试用的目标编码器速度，一般用于基本调参



void shiki_init(void);

void pit_handler_0(void);
void pit_handler_1(void);
void pit_handler_2(void);
void pit_handler_3(void);

extern int time_count_10ms;				// PIT3 周期为10ms的计数器，发给vofa用作x轴

uint32_t pit_get_millis(void);

#endif
