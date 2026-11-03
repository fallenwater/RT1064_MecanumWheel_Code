#include "zf_common_typedef.h"
#include "zf_common_fifo.h"
#include "zf_common_interrupt.h"
#include "zf_driver_delay.h"
#include "uart.h"



uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8 get_data = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;


//uint8 gpio_status;


// UART1 串口接收中断处理
void uart_rx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART1_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
}


// 参数： 1.UART索引		2.对应UART的接收中断向量		3.对应UART的接收中断优先级
void shiki_uart_init(uart_index_enum uart_index, IRQn_Type irqn, uint8 priority)
{
	
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

	uart_init(uart_index, UART1_BAUDRATE, UART1_TX_PIN, UART1_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
	uart_rx_interrupt(uart_index, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
	interrupt_set_priority(irqn, priority);                                   // 设置对应 UART_INDEX 的中断优先级为 0

	uart_write_string(uart_index, "UART Text.");                                // 输出测试信息
	uart_write_byte(uart_index, '\r');                                          // 输出回车
	uart_write_byte(uart_index, '\n');                                          // 输出换行	
}
	


void uart_send_test(void)
{
    uart_pkg pkg;

    pkg.head = 0xAA;   // 帧头
    pkg.a = 12;
    pkg.b = 34;
    pkg.c = 56;
    pkg.d = 78;
    pkg.tail = 0x55;   // 帧尾

    uart_write_buffer(UART1_INDEX, (uint8 *)&pkg, sizeof(pkg));
}









/*-------------------逐飞测试代码----------------*/
void uart1_init()
{
	fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区

	uart_init(UART1_INDEX, UART1_BAUDRATE, UART1_TX_PIN, UART1_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
	uart_rx_interrupt(UART1_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
	interrupt_set_priority(UART1_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

	uart_write_string(UART1_INDEX, "UART Text.");                                // 输出测试信息
	uart_write_byte(UART1_INDEX, '\r');                                          // 输出回车
	uart_write_byte(UART1_INDEX, '\n');                                          // 输出换行	
	
}


// 读取UART1_RX 接收到的数据并回显（通过串口1发送回去）
void uart1_test()				
{
	fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
	if(fifo_data_count != 0)                                                // 读取到数据了
	{
		fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
		uart_write_string(UART1_INDEX, "\r\nUART1 get data:");                // 输出测试信息
		uart_write_buffer(UART1_INDEX, fifo_get_data, fifo_data_count);      // 将读取到的数据发送出去
	}
	system_delay_ms(10);
}
