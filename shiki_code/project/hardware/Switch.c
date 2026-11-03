
/* 此处代码用于主板按键与拨码开关函数调用与变量记录 */

#include "Switch.h"
#include "UI.h"

bool button_enter_flag = 0;								// enter 按键标志位，表明该按键被按下，需要执行对应指令
bool button_back_flag  = 0;								// back  按键标志位
bool button_up_flag    = 0;								// up    按键标志位 
bool button_down_flag  = 0;								// down  按键标志位

int8 UI_Read_DIP_Switch_to_mode(void)
{
	int8 DIP_Switch_1 = gpio_get_level(DIP_SWITCH_1);
	int8 DIP_Switch_2 = gpio_get_level(DIP_SWITCH_2);
	int8 DIP_Switch_3 = gpio_get_level(DIP_SWITCH_3);
	int8 mode = 0;
	if (!DIP_Switch_1)											mode = 0;
	else if (DIP_Switch_1 && !DIP_Switch_2)						mode = 1;
	else if (DIP_Switch_1 &&  DIP_Switch_2 && !DIP_Switch_3)	mode = 2;
	return mode;
}

void UI_Read_Button(void)
{
	if (KEY_SHORT_PRESS == key_get_state(KEY_1))
	{// 确定 enter
		switch (UI_Mode)
		{
			case UI_MODE_MENU:		Button_To_Menu(BUTTON_BACK);	break;
			default:												break;
		}
		key_clear_state (KEY_1);
	}
	if (KEY_SHORT_PRESS == key_get_state(KEY_2))
	{// 返回 back
		switch (UI_Mode)
		{
			case UI_MODE_MENU:		Button_To_Menu(BUTTON_ENTER);	break;
			default:												break;
		}
		key_clear_state (KEY_2);
	}
	if (KEY_SHORT_PRESS == key_get_state(KEY_3))
	{//向上 up
		switch (UI_Mode)
		{
			case UI_MODE_MENU:		Button_To_Menu(BUTTON_UP);		break;
			default:												break;
		}
		key_clear_state (KEY_3);
	}
	if (KEY_SHORT_PRESS == key_get_state(KEY_4))
	{//向下 down
		switch (UI_Mode)
		{
			case UI_MODE_MENU:		Button_To_Menu(BUTTON_DOWN);	break;
			default:												break;
		}
		key_clear_state (KEY_4);
	}
	
	
}

void Button_To_Menu(button_mode_enum button_mode)
{
	menu_refresh_flag = 1;								// 将按键刷新标志位置1，刷新菜单界面
	
	if      (button_mode == BUTTON_ENTER)				// 确认键反馈
	{
		button_enter_flag = 1;
	}
	else if (button_mode == BUTTON_BACK)				// 返回键键反馈
	{
		button_back_flag = 1;
	}
	else if (button_mode == BUTTON_UP)					// 上键反馈
	{
		button_up_flag	 = 1;
	}
	else if (button_mode == BUTTON_DOWN)				// 下键反馈
	{
		button_down_flag = 1;
		
	}
	
}

void Key_Switch_init(uint32 period)
{
	// 按键初始化，扫描周期15ms
	key_init (period);										
	// 拨码开关初始化
	gpio_init(DIP_SWITCH_1, GPI, GPIO_HIGH, GPI_PULL_UP);
    gpio_init(DIP_SWITCH_2, GPI, GPIO_HIGH, GPI_PULL_UP);
	gpio_init(DIP_SWITCH_3, GPI, GPIO_HIGH, GPI_PULL_UP);
}
