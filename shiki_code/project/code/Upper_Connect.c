#include "Upper_Connect.h"
#include "uart.h"


// UART1 串口接收中断处理
//void uart_rx_interrupt_handler (void)
//{ 
////    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
//    uart_query_byte(UART1_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
//    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
//}
