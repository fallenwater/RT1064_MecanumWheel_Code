#ifndef _SWITCH_H_
#define _SWITCH_H_

#include "zf_common_typedef.h"
#include "zf_device_key.h"

#define DIP_SWITCH_1			(C26)
#define DIP_SWITCH_2			(C27)
#define DIP_SWITCH_3			(D27)
#define BUTTON_1				(C12)
#define BUTTON_2				(C14)
#define BUTTON_3				(C13)
#define BUTTON_4				(C15)

typedef enum {
	BUTTON_ENTER  = 1,
	BUTTON_BACK   = 2,
	BUTTON_UP	  = 3,
	BUTTON_DOWN   = 4
} button_mode_enum;				// 按键内容定义枚举


int8 UI_Read_DIP_Switch_to_mode(void);
void Button_To_Menu(button_mode_enum button_mode);
void UI_Read_Button(void);
void Key_Switch_init(uint32 period);

extern bool button_enter_flag;								// enter 按键标志位，表明该按键被按下，需要执行对应指令
extern bool button_back_flag ;								// back  按键标志位 
extern bool button_up_flag	 ;								// up    按键标志位
extern bool button_down_flag ;								// down  按键标志位

#endif 
