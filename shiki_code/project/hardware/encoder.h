#ifndef _ENCODER_H_
#define _ENCODER_H_

#include "zf_common_typedef.h"
#include <stdint.h> 


// **************************** 重定义区域 ****************************
//#define ENCODER_1_TIMER             (QTIMER1_ENCODER1)
//#define ENCODER_1_A                 (QTIMER1_ENCODER1_CH1_C0)
//#define ENCODER_1_B                 (QTIMER1_ENCODER1_CH2_C1)

//#define ENCODER_2_TIMER             (QTIMER1_ENCODER2)
//#define ENCODER_2_A                 (QTIMER1_ENCODER2_CH1_C2)
//#define ENCODER_2_B                 (QTIMER1_ENCODER2_CH2_C24)

//#define ENCODER_3_TIMER             (QTIMER2_ENCODER1)
//#define ENCODER_3_A                 (QTIMER2_ENCODER1_CH1_C3)
//#define ENCODER_3_B                 (QTIMER2_ENCODER1_CH2_C25)

//#define ENCODER_4_TIMER             (QTIMER3_ENCODER1)
//#define ENCODER_4_A                 (QTIMER3_ENCODER1_CH1_B16)
//#define ENCODER_4_B                 (QTIMER3_ENCODER1_CH2_B17)

#define ENCODER_1_TIMER             (QTIMER2_ENCODER1)
#define ENCODER_1_A                 (QTIMER2_ENCODER1_CH1_C3)
#define ENCODER_1_B                 (QTIMER2_ENCODER1_CH2_C4)

#define ENCODER_2_TIMER             (QTIMER2_ENCODER2)
#define ENCODER_2_A                 (QTIMER2_ENCODER2_CH1_C5)
#define ENCODER_2_B                 (QTIMER2_ENCODER2_CH2_C25)

#define ENCODER_3_TIMER             (QTIMER1_ENCODER1)
#define ENCODER_3_A                 (QTIMER1_ENCODER1_CH1_C0)
#define ENCODER_3_B                 (QTIMER1_ENCODER1_CH2_C1)

#define ENCODER_4_TIMER             (QTIMER1_ENCODER2)
#define ENCODER_4_A                 (QTIMER1_ENCODER2_CH1_C2)
#define ENCODER_4_B                 (QTIMER1_ENCODER2_CH2_C24)

#define PIT_ENCODER                 (PIT_CH0)                                 // 使用的周期中断编号 如果修改 需要同步对应修改周期中断编号与 isr.c 中的调用



typedef enum {
    ENCODER_NUM_1,  // 编码器1
    ENCODER_NUM_2,  // 编码器2
    ENCODER_NUM_3,  // 编码器3
    ENCODER_NUM_4,  // 编码器4
	ENCODER_COUNT   // 编码器总数（自动计算，用于参数校验）
} EncoderNum;


/**
 * 编码器初始化相关函数
 */
void Encoder_Init(void);                  // 初始化所有编码器

/**
 * 编码器数据读取相关函数
 */
int32_t Encoder_ReadSpeed(EncoderNum num);  // 读取指定编码器的速度值



#endif
