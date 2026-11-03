/* **************************************************************************************************
*****************************************************************************************************
***************																		  ***************	
***************		     因为无线串口完全用于vofa，所以代码已经全部移植到vofa.c		  ***************
***************																		  ***************
*****************************************************************************************************
************************************************************************************************** */


//#include "zf_common_typedef.h"
//#include "wireless_uart.h"
//#include "zf_common_headfile.h"


//volatile int32_t vofa_enc_snapshot[4];
//volatile bool vofa_enc_ready = false;

//void my_wireless_init(void)
//{
//	
//	wireless_uart_send_byte('\r');
//    wireless_uart_send_byte('\n');
//    wireless_uart_send_string("SEEKFREE wireless uart demo.\r\n");              // 初始化正常 输出测试信息
//	
//	if(wireless_uart_init())                                                    // 判断是否通过初始化
//	{
//		while(1)                                                                // 初始化失败就在这进入死循环
//		{
//			gpio_toggle_level(CoreBoard_LED);                                            // 翻转 LED 引脚输出电平 控制 LED 亮灭
//			system_delay_ms(2000);                                               // 短延时快速闪灯表示异常
//		}
//	}
//}

//void vofa_send_task(void)
//{
//    if (!vofa_enc_ready) return;
//    vofa_enc_ready = false;

//    int32_t a = vofa_enc_snapshot[0];
//    int32_t b = vofa_enc_snapshot[1];
//    int32_t c = vofa_enc_snapshot[2];
//    int32_t d = vofa_enc_snapshot[3];
//	
////	float v0, v1, v2, v3;

//    char buf[128];

//    // ? 带前缀 channels:
//    int len = snprintf(buf, sizeof(buf),
//                       "channels:%ld,%ld,%ld,%ld\n",
//                       (long)a, (long)b, (long)c, (long)d);

//	
////	int len1 = snprintf(buf, sizeof(buf),
////                       ":%.3f,%.3f,%.3f,%.3f\n",
////                       v0, v1, v2, v3);
//	
//    // 或者：不带前缀
//    // int len = snprintf(buf, sizeof(buf),
//    //                    "%ld,%ld,%ld,%ld\n",
//    //                    (long)a, (long)b, (long)c, (long)d);

//    if (len > 0)
//    {
//		wireless_uart_send_string(buf);
//        
//    }
//}