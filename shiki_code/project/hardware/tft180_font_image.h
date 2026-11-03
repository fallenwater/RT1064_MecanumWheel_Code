
#ifndef _TFT180_FONT_IMAGE_H_
#define _TFT180_FONT_IMAGE_H_

#include "zf_common_font.h"
#include "zf_common_typedef.h"


extern const uint8      ascii_font_8x16[][16];
extern const uint8      ascii_font_6x8[][6];
extern const uint8      chinese_test[8][16];
extern const uint8      oled_16x16_chinese[][16];

// 一级菜单文字
extern const uint8 		font_huitui[4][16];							// 回退
extern const uint8 		font_qidong[4][16];							// 启动
extern const uint8 		font_suduhuanxiangying[10][16];				// 速度环响应
extern const uint8 		font_tuoluoyi[6][16];						// 陀螺仪
extern const uint8 		font_tiaoshimoshi[8][16];					// 调试模式
extern const uint8 		font_dangqianmoshi[10][16];					// 当前模式


// 二级菜单文字
extern const uint8 		font_youxijinxingzhong[10][16];				// 游戏进行中
extern const uint8 		font_youxizanting[8][16];					// 游戏暂停
extern const uint8 		font_youxizantingzhong[10][16]; 			// 游戏暂停中
extern const uint8 		font_youxijixu[8][16];						// 游戏继续
extern const uint8 		font_xiangduiweizhihuantiaoshi[14][16];		// 相对位置环调试
extern const uint8 		font_jueduiweizhihuantiaoshi[14][16];		// 绝对位置环调试
extern const uint8 	 	font_bisaimoshi[8][16];						// 比赛模式
extern const uint8 		font_weizhihuantiaoshi[10][16];				// 位置环调试
extern const uint8 	 	font_chunsuduhuantiaoshi[12][16];			// 纯速度环调试
extern const uint8 	 	font_zhuanxianghuantiaoshi[10][16];			// 转向环调试
extern const uint8 	 	font_tingzhimoshi[8][16];					// 停止模式

// 建图文字
extern const uint8 		font_ditu[4][16];							// 地图

// 变量表文字
extern const uint8 		font_bianliangbiao[6][16];					// 变量表 

// 汉字
extern const uint8 		font_yuanshenniubi[8][16];					// 原神牛逼
extern const uint8 		font_xuebaokeji[8][16];						// 雪豹科技

// 图像
extern const uint8      gImage_shiki[35584];						// 神山识
extern const uint8      gImage_xuebao[32768];						// 芝士雪豹

#endif
