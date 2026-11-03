#ifndef _UART_H_
#define _UART_H_

#include "zf_driver_uart.h"
#include "zf_common_typedef.h"
#include "zf_common_fifo.h"
#define UART1_INDEX              (UART_1)                           // 默认 UART_1
#define UART1_BAUDRATE           (115200)                           // 默认 115200
#define UART1_TX_PIN             (UART1_TX_B12)                     // 默认 UART1_TX_A9
#define UART1_RX_PIN             (UART1_RX_B13)                     // 默认 UART1_RX_A10

#define UART2_INDEX              (UART_2)                           // 默认 UART_1
#define UART2_BAUDRATE           (115200)                           // 默认 115200
#define UART2_TX_PIN             (UART2_TX_B18)                     // 默认 UART1_TX_A9
#define UART2_RX_PIN             (UART2_RX_B19)                     // 默认 UART1_RX_A10

#define UART1_PRIORITY           (LPUART1_IRQn)                     // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体
#define UART2_PRIORITY           (LPUART2_IRQn)                     // 对应串口中断的中断编号 在 MIMXRT1064.h 头文件中查看 IRQn_Type 枚举体


typedef struct
{
    uint8_t head;   // 包头
    uint8_t a;
    uint8_t b;
    uint8_t c;
    uint8_t d;
    uint8_t tail;   // 包尾
} uart_pkg;


typedef enum {
	UART_USING
} UART_STATE;


void uart1_init(void);
void uart_send_test(void);
void uart1_test(void);

extern uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
extern uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

extern uint8 get_data ;                                                             // 接收数据变量
extern uint32 fifo_data_count ;                                                     // fifo 数据个数

extern fifo_struct uart_data_fifo;



void shiki_uart_init(uart_index_enum uart_index, IRQn_Type irqn, uint8 priority);





#endif
