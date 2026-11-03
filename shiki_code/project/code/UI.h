#ifndef _UI_H_
#define _UI_H_

#include "zf_common_typedef.h"

/* ================================= 宏定义声明区 ================================= */
#define MENU_LAYER_NUM				(3)				// 菜单层级数，记录数组
#define MENU_LAYER_1_MAX_ROW		(5)				// 一级菜单行数
#define MENU_LAYER_2_0_MAX_ROW	(2)				// 一级菜单选项0->二级菜单行数
#define MENU_LAYER_2_1_MAX_ROW	(7)				// 一级菜单选项1->二级菜单行数
#define MENU_LAYER_2_2_MAX_ROW	(3)				// 一级菜单选项2->二级菜单行数
#define MENU_LAYER_2_3_MAX_ROW	(3)				// 一级菜单选项3->二级菜单行数




/* ================================= 枚举声明区 ================================= */
typedef enum {
	UI_MODE_MENU = 0,				// 菜单
	UI_MODE_READ_Var= 1,			// 变量读取
	UI_MODE_MAP = 2					// 建图
} ui_mode;											// TFT180 ui模式枚举

typedef enum {
	MENU_LAYER_1 = 0,
	MENU_LAYER_2 = 1,
	MENU_LAYER_3 = 2
} menu_layer_enum;									// menu层级变量枚举

typedef enum {
	START = 0,
	MODE_SELECT  = 1,
	Vel_PID_menu = 2,
	TUOLUOYI = 3
} menu_layer_1_var_enum;							// menu层级变量枚举
/* ================================= 结构体声明区 ================================= */



/* ================================= 函数声明区 ================================= */
void menu_1(void);
void menu_2(void);
void menu_3(void);
void menu(void);
void Read_Var_UI(void);
void Map_UI(void);
void UI(void);

void Menu_to_Motor_Enable_Manage(void);
void menu_2_task_logic(void);				// 二级菜单下各选项任务


void ui_menu_1_start(bool start_flag);
void ui_menu_1_mode_select(void);
void ui_menu_1_vel(uint8 state_flag);
void ui_default_genshit(void);
void ui_menu_default_genshit(void);

/* ================================= 变量声明区 ================================= */
/* ----------     公共变量     ----------*/
extern uint8 UI_Mode;



/* ---------- 默认调参菜单变量 ----------*/
extern int8 menu_layer_row_array[MENU_LAYER_NUM];
extern uint8 menu_layer;							// 菜单层级（指向菜单当前处在第几层）

extern bool menu_refresh_flag;						// 菜单刷新标志位


/* ---------- 调试读参菜单 ----------*/



/* ---------- 建图UI ----------*/

#endif 
