#include "ui.h"
#include "zf_device_key.h"
#include "zf_device_tft180.h"
#include "zf_common_font.h"
#include "tft180_font_image.h"
#include "Switch.h"
#include "sb.h"
#include "PID_Control.h"
#include "imu660ra.h"
/* ================================= 变量定义区 ================================= */
/* ----------     公共变量     ----------*/
uint8 UI_Mode = 0;
static uint8 last_UI_Mode = 255; 			// 初始值设为一个不存在的模式，保证第一次上电能刷新

/* ---------- 默认调参菜单变量 ----------*/
uint8 menu_layer_1_row = 0;					// 记录一级菜单行数
uint8 menu_layer_2_row = 0;					// 记录二级菜单行数
uint8 menu_layer_3_row = 0;					// 记录三级菜单行数

int8  menu_layer_row_array[MENU_LAYER_NUM] = {0, 0, 0};
int8  menu_layer_2_max_row_array[MENU_LAYER_1_MAX_ROW] = {	MENU_LAYER_2_0_MAX_ROW,
															MENU_LAYER_2_1_MAX_ROW,
															MENU_LAYER_2_2_MAX_ROW,
															MENU_LAYER_2_3_MAX_ROW,};		// 记录二级菜单各界面的按键行数
int8  menu_layer_3_max_row_array[MENU_LAYER_1_MAX_ROW] = { 0 };								// 记录三级菜单各界面的按键行数（暂无想法）

uint8 menu_layer = 0;							// 菜单层级（指向菜单当前处在第几层）

bool menu_refresh_flag = 1;						// 菜单刷新标志位，减少主循环调用量，菜单无需不断刷新，只在读取按键按下时才刷新画面

static bool game_start_flag	  = 0;				// 菜单2 游戏开始标志位，默认是0，一切换到 启动子菜单就开始游戏
static bool general_used_flag = 0;				// 通用标志位
// 引入静态变量检测按键边沿
static bool last_game_start_flag = 0;


// 根据你在 shiki_init 中定义的颜色
#define MENU_TEXT_COLOR     RGB565_BLUE   // 正常文字颜色
#define MENU_BACK_COLOR     RGB565_WHITE  // 正常背景颜色 (高亮时的文字颜色)

static const char * const mode_name[] = {		// 模式名
    "STOP",        // 停止
	"RACE",        // 比赛
	"CAM_OFF",     // MODE_DEBUG_CAM_OFFSET
    "SER_TGT",     // 绝对位置
	"POS_PID",		// 位姿
    "ANG_PID",      // MODE_ANGLE_PID_TEST
    "NVEL",        // MODE_DEBUG_NORMAL_VEL
};


/* ---------- 调试读参菜单 ----------*/



/* ---------- 建图UI ----------*/



/* ================================= 函数定义区 ================================= */
// 菜单目录
void UI(void)
{	
	UI_Mode = UI_Read_DIP_Switch_to_mode();
	
	bool is_mode_changed = (UI_Mode != last_UI_Mode);	
	if (is_mode_changed)
	{
		// 如果刚刚切换到菜单模式，强制置位刷新标志，解决“切回来不显示”的问题
		if (UI_Mode == UI_MODE_MENU) {
			menu_refresh_flag = 1; 		// 菜单刷新标志位
		}
		// 如果刚刚切换到变量表模式，我们需要通知 Read_Var_UI 清屏一次(通过下面传参或全局变量，这里推荐用下面的静态变量法)
	}
	// ===============================
	
	switch(UI_Mode)										
	{
		case UI_MODE_MENU:		menu();			break;		// 菜单UI
		case UI_MODE_MAP:		Map_UI();		break;		// 建图UI
		case UI_MODE_READ_Var:	Read_Var_UI();	break;		// 变量UI
		default: 		 ui_default_genshit();	break;		// 错误UI
	}
	
	// 把更新历史记录的操作放在最后！
	// 这样上面的 Read_Var_UI 运行时，因为 last_UI_Mode 还没变，所以能触发清屏
	last_UI_Mode = UI_Mode; // 更新记录
	
}

void Map_UI(void)
{
	tft180_clear();													// 刷新画面
	tft180_show_chinese(8*4,  16*0,   16, (const uint8 *)font_ditu, 2, RGB565_BLUE);
}

void Read_Var_UI(void)
{	
	// === 静态层：只在刚切换进这个模式时执行一次 ===
	if (last_UI_Mode != UI_Mode) 
	{
		last_UI_Mode = UI_Mode; // 更新状态
		
		tft180_clear();	// 只在这里清屏！防止闪烁
		
		// 绘制那些不会变的汉字和标签
		tft180_show_chinese(8*4, 16*0, 16, (const uint8 *)font_bianliangbiao, 3, RGB565_BLUE);
		tft180_show_string( 0,   16*1, "LF:");
		tft180_show_string( 8*9, 16*1, "RF:");
		tft180_show_string( 0,   16*2, "LB:");
		tft180_show_string( 8*9, 16*2, "RB:");
	}
	// === 动态层：实时刷新数据 ===
	// 注意：为了防止数字重叠，show_int 函数最好有背景色覆盖功能
	// 如果没有背景色覆盖，可以在显示数字前，先在那个位置显示几个空格字符串来"擦除"旧数字
	
	// 编码器实时数据
	tft180_show_string(  8*5,   16*1, "   ");
	tft180_show_string( 8*12,   16*1, "   ");
	tft180_show_string(  8*5,   16*2, "   ");
	tft180_show_string( 8*12,   16*2, "   ");
	tft180_show_int(8*3,  16*1,  Read_EncoderSpeed[0],    3);                     // 显示 int16 数据
	tft180_show_int(8*12,  16*1,  Read_EncoderSpeed[1],    3);                     // 显示 int16 数据
	tft180_show_int(8*3,  16*2,  Read_EncoderSpeed[2],    3);                     // 显示 int16 数据
	tft180_show_int(8*12,  16*2,  Read_EncoderSpeed[3],    3);                     // 显示 int16 数据
	
	// 陀螺仪数据
	tft180_show_string(0,16*3,"yaw_ag:");      		tft180_show_float(8*7, 16*3 , Gyro_Angle.Zdata, 3, 4);

	// 编码器累计数据
	tft180_show_string( 8*0,   16*4, "               ");
	tft180_show_string( 8*0,   16*5, "               ");
	tft180_show_int(8*0,  16*4,  sum_EncoderSpeed[0],    7);                     // 显示 int16 数据
	tft180_show_int(8*8,  16*4,  sum_EncoderSpeed[1],    7);                     // 显示 int16 数据
	tft180_show_int(8*0,  16*5,  sum_EncoderSpeed[2],    7);                     // 显示 int16 数据
	tft180_show_int(8*8,  16*5,  sum_EncoderSpeed[3],    7);                     // 显示 int16 数据

	// 当前X,Y,yaw
	tft180_show_string( 8*0,   16*6, "               ");
	tft180_show_string( 8*0,   16*7, "               ");
	tft180_show_string( 8*0,   16*8, "               ");
	tft180_show_int(8*0,  16*6,  current_pose.x,    7);                     // 显示 int16 数据
	tft180_show_int(8*0,  16*7,  current_pose.y,    7);                     // 显示 int16 数据
	tft180_show_int(8*0,  16*8,  current_pose.yaw,    7);                     // 显示 int16 数据

}
void menu(void)
{
	// 如果没有按键按下，直接返回
    if (!menu_refresh_flag) return;
	
	// 读取按下按键并判断逻辑
	if (button_enter_flag == 1)						// 检测到处于一级菜单并且到按下enter键
	{
		if (menu_layer == MENU_LAYER_1)
		{
			menu_layer = MENU_LAYER_2;	
			menu_layer_row_array[MENU_LAYER_2] = 0; 									// 默认选中第0行
			if( menu_layer_row_array[MENU_LAYER_1] == START)	game_start_flag = 1;	// 只有在一级菜单按下“启动”后，默认直接开启
			else 												game_start_flag = 0;
		}
		else if (menu_layer == MENU_LAYER_2)					// 二级菜单下 检测到enter\back 按下
		{
			menu_2_task_logic();
		}
	}
	// 检测到不处于一级菜单，并且按下back键，回退到上一层
	else if (button_back_flag == 1 && menu_layer != MENU_LAYER_1)					
	{
		menu_layer_row_array[menu_layer] = 0;										// 将当前层级菜单的 行数 跳回第0行
		menu_layer = menu_layer -1;													// 回退到上一层级数
		game_start_flag = 0;														// 不管处于哪个二级菜单，都保证暂停游戏
		
	}
	// 检测到“向上”按键按下
	else if (button_up_flag == 1)													
	{
		uint8 max_row = 0;
		// 根据当前层级判断最大行数
        if	   (menu_layer == MENU_LAYER_1) max_row = MENU_LAYER_1_MAX_ROW;
        else if(menu_layer == MENU_LAYER_2) max_row = menu_layer_2_max_row_array[menu_layer_row_array[MENU_LAYER_1]];

        menu_layer_row_array[menu_layer] = (menu_layer_row_array[menu_layer] > 0) ? 
                                             (menu_layer_row_array[menu_layer] - 1) : 
                                             (max_row - 1);											// 锁定上一级菜单第几个选项，并跳转对应选项对应的最大值
	}
	// 检测到“向下”按键按下
	else if (button_down_flag == 1)													
	{
		uint8 max_row = 0;
		// 根据当前层级判断最大行数
        if(menu_layer == MENU_LAYER_1) max_row = MENU_LAYER_1_MAX_ROW;
        else if(menu_layer == MENU_LAYER_2) max_row = menu_layer_2_max_row_array[menu_layer_row_array[MENU_LAYER_1]];

        menu_layer_row_array[menu_layer] = (menu_layer_row_array[menu_layer] < max_row - 1) ? 
                                             (menu_layer_row_array[menu_layer] + 1) : 
                                             (0);
	}
	

	// 将电机控制逻辑放在按键逻辑之后，确保 Flag 变化后立即生效
    Menu_to_Motor_Enable_Manage();			// 独立判断电机状态，避免冲突
	
	// 清理按键标志位
	button_enter_flag = 0;
	button_back_flag  = 0;
	button_up_flag    = 0;
	button_down_flag  = 0;
	menu_refresh_flag = 0;											// 清空菜单刷新标志位，下次按下按键才会继续刷新

	// --------- 绘图部分 ----------（只显示画面，不处理数据）
	tft180_clear();													// 刷新画面
	switch(menu_layer)												// 判断当前处理哪一级菜单
	{
		case MENU_LAYER_1:		menu_1(); 									break;	// 一级菜单
		case MENU_LAYER_2:		menu_2();									break;	// 二级菜单
		case MENU_LAYER_3:		menu_3();									break;	// 三级菜单
		default: 					 	 									break;	
	}
}

// 一级菜单UI显示
void menu_1(void)			
{
	tft180_show_string (8*0,  16* menu_layer_row_array[MENU_LAYER_1], ">"); 
	tft180_show_chinese(8*1,  16*0,   16, (const uint8 *)font_qidong, 		 	 2, RGB565_BLUE);           // 启动
	tft180_show_chinese(8*1,  16*1,   16, (const uint8 *)font_tiaoshimoshi,		 4, RGB565_BLUE);           // 调试模式选择
	tft180_show_chinese(8*1,  16*2,   16, (const uint8 *)font_suduhuanxiangying, 5, RGB565_BLUE);           // 速度环调参
	tft180_show_chinese(8*1,  16*3,   16, (const uint8 *)font_tuoluoyi, 		 3, RGB565_BLUE);           // 陀螺仪选择
	
	tft180_show_chinese(8*0,  16*7,   16, (const uint8 *)font_dangqianmoshi, 	 5, RGB565_BLUE);           // 当前模式
	tft180_show_string (8*9,  16*7,   mode_name[Current_Car_Action_Mode]); 
}

// 二级菜单UI显示
void menu_2(void)
{
	int8 last_layer_row = menu_layer_row_array[MENU_LAYER_1];					// 记录进入二级菜单前的行数，显示对应菜单的ui画面
	
	tft180_show_string (8*3,  16* menu_layer_row_array[MENU_LAYER_2], ">"); 
	switch (last_layer_row)
	{
		case START:			ui_menu_1_start(game_start_flag);	break;
		case MODE_SELECT:	ui_menu_1_mode_select();			break;	
		case Vel_PID_menu:	ui_menu_default_genshit();			break;	
		default:			ui_menu_default_genshit();			break;

	}
}
	

// 三级菜单UI显示
void menu_3(void)
{
	tft180_show_chinese(8*4,  16*4,   16, (const uint8 *)font_yuanshenniubi, 4, RGB565_BLUE);           // 原神牛逼
}


/* ================================= 数据处理区 ================================= */
// 电机使能管理函数
void Menu_to_Motor_Enable_Manage(void)
{
	// 安全条件判断：
    // 1. 必须在二级菜单
    // 2. 必须是从“启动(START)”菜单进来的
    // 3. 必须 game_start_flag 为 1
    bool safe_to_run = (menu_layer == MENU_LAYER_2) && 
                       (menu_layer_row_array[MENU_LAYER_1] == START) && 
                       (game_start_flag == 1);

    if (safe_to_run)	// 允许运动
    {
		SPEED_ENABLE = true;					// 使能控制
		if (Current_Car_Action_Mode == MODE_DEBUG_NORMAL_VEL)
		{
//			target_pose.x = 2;
//			target_pose.y = 0;
//			target_pose.yaw = 0;
			target_pose.yaw = 0;
			hangle_keep_pid.integral = 0;
			hangle_keep_pid.last_err = 0;
			for(int i = 0; i < 4; i ++)
			{
				hvel_pid[i].output = 0;
				hvel_pid[i].error_now = 0;
				hvel_pid[i].error_last = 0;
				Target_EncoderSpeed[i] = -Set_Motor_Speed.Motor_Low_Speed;
//				Target_PPS[i] = -Set_Motor_Speed.Motor_Low_Speed;
			}
			
//			hpos_x_pid.integral = 0;		// 积分清零
//			hpos_y_pid.integral = 0;		// 积分清零
//			hangle_keep_pid.integral = 0;
		}
		else if (Current_Car_Action_Mode == MODE_ANGLE_PID_TEST)
		{
			target_pose.yaw = 0;
			hangle_keep_pid.integral = 0;
			hangle_keep_pid.last_err = 0;
			for(int i = 0; i < 4; i ++)
			{
				hvel_pid[i].output = 0;
				hvel_pid[i].error_now = 0;
				hvel_pid[i].error_last = 0;
			}
		}
		else if (Current_Car_Action_Mode == MODE_POSE_PID_TEST)
		{
			target_pose.x = 0;
			target_pose.y = 0;
			target_pose.yaw = 0;
			hpos_x_pid.integral = 0;		// 积分清零
			hpos_y_pid.integral = 0;		// 积分清零
			hangle_keep_pid.integral = 0;
			hangle_keep_pid.last_err = 0;
			for(int i = 0; i < 4; i ++)
			{
				hvel_pid[i].output = 0;
				hvel_pid[i].error_now = 0;
				hvel_pid[i].error_last = 0;
			}
		}
		else if (Current_Car_Action_Mode == MODE_RACE_FULL)
		{
			
		}
		else if (Current_Car_Action_Mode == MODE_DEBUG_SERIAL_TARGET)
		{
			
		}
		else if (Current_Car_Action_Mode == MODE_DEBUG_CAM_OFFSET)
		{
			
		}	
        
    }
    else				// 任意条件未满足，强制停止
    {
        for(int i = 0; i < 4; i ++)
        {
			SPEED_ENABLE = false;
            Target_EncoderSpeed[i] = 0;	
        }
    }
}

// 二级菜单下各选项的任务与数据处理
void menu_2_task_logic(void)	
{
	int8 last_layer_row = menu_layer_row_array[MENU_LAYER_1]; // 一级菜单选了谁
    int8  now_layer_row = menu_layer_row_array[MENU_LAYER_2]; // 二级菜单当前在哪行
	
	if (last_layer_row == START)								// 处于二级菜单“启动”下，并且检测到按键按下
	{
		if 	    (now_layer_row == 0)	 	game_start_flag = !game_start_flag;				// 取反·开始和暂停游戏任务来回
		else if	(now_layer_row == 1) 	   {menu_layer_row_array[menu_layer--] = 0; 		// 清空当前级数的行数归为0，并回到上一级
											game_start_flag = 0;					}		// 清空开始标志位		
	}
	else if(last_layer_row == MODE_SELECT)
	{  
		if 	    (now_layer_row !=0)		Current_Car_Action_Mode = now_layer_row;			// 如果按下时选项不是0，则配置模式
		menu_layer_row_array[menu_layer--] = 0;												// 无论在哪按下enter，都退出当前界面
		 										
	}
	else if(last_layer_row == Vel_PID_menu)
	{
		if 	    (now_layer_row == 0)		menu_layer_row_array[menu_layer--] = 0;			// 清空当前级数的行数归为0，并回到上一级
	}
	else if(last_layer_row == TUOLUOYI)
	{
		if 	    (now_layer_row == 0)		menu_layer_row_array[menu_layer--] = 0;	// 清空当前级数的行数归为0，并回到上一级
	}
	else					// 如果是还未定义的二级菜单，一样可以在第一行退回一级菜单
	{
		if 	    (now_layer_row == 0)		menu_layer_row_array[menu_layer--] = 0;	// 清空当前级数的行数归为0，并回到上一级
	}
	
	
}

void menu_2_task_start(bool start_flag)
{
	
}
// 一级菜单“启动”的子菜单UI
void ui_menu_1_start(bool start_flag)
{
	if (start_flag == 0) {
		tft180_show_chinese(8*4,  16*0,   16, (const uint8 *)font_youxijixu, 4, RGB565_BLUE);           	// 游戏继续
		tft180_show_chinese(8*4,  16*1,   16, (const uint8 *)font_huitui, 2, RGB565_BLUE);           		// 回退
		tft180_show_chinese(8*4,  16*5,   16, (const uint8 *)font_youxizantingzhong, 5, RGB565_BLUE);       // 游戏暂停中
	}else if (start_flag == 1){
		tft180_show_chinese(8*4,  16*0,   16, (const uint8 *)font_youxizanting, 4, RGB565_BLUE);           	// 游戏暂停
		tft180_show_chinese(8*4,  16*1,   16, (const uint8 *)font_huitui, 2, RGB565_BLUE);           		// 回退
		tft180_show_chinese(8*4,  16*5,   16, (const uint8 *)font_youxijinxingzhong, 5, RGB565_BLUE);       // 游戏进行中
	}
	
	tft180_show_chinese(8*0,  16*7,   16, (const uint8 *)font_dangqianmoshi, 	 5, RGB565_BLUE);           // 当前模式
	tft180_show_string (8*9,  16*7,   mode_name[Current_Car_Action_Mode]);          						 
}

// 一级菜单“速度环响应”的子菜单UI
void ui_menu_1_mode_select(void)
{
	tft180_show_chinese(8*4,  16*0,   16, (const uint8 *)font_huitui, 2, RGB565_BLUE);           			// 回退
	tft180_show_chinese(8*4,  16*1,   16, (const uint8 *)font_bisaimoshi, 4, RGB565_BLUE);           		// 比赛模式
	tft180_show_chinese(8*4,  16*2,   16, (const uint8 *)font_xiangduiweizhihuantiaoshi, 4, RGB565_BLUE);   // 相对位置环
	tft180_show_chinese(8*4,  16*3,   16, (const uint8 *)font_jueduiweizhihuantiaoshi, 4, RGB565_BLUE);     // 绝对位置环
	tft180_show_chinese(8*4,  16*4,   16, (const uint8 *)font_weizhihuantiaoshi, 5, RGB565_BLUE);     		// 位置环调试
	tft180_show_chinese(8*4,  16*5,   16, (const uint8 *)font_zhuanxianghuantiaoshi, 4, RGB565_BLUE);       // 转向环
	tft180_show_chinese(8*4,  16*6,   16, (const uint8 *)font_chunsuduhuantiaoshi, 4, RGB565_BLUE);         // 纯速度环
//	tft180_show_chinese(8*4,  16*6,   16, (const uint8 *)font_tingzhimoshi, 4, RGB565_BLUE);         		// 停止模式
}

// 一级
void ui_menu_1_debug()
{
	
}
void ui_default_genshit(void)
{
	tft180_show_chinese(8*4,  16*4,   16, (const uint8 *)font_yuanshenniubi, 4, RGB565_BLUE);           // 原神牛逼
}

// 一级菜单中“未定义UI”的默认子菜单
void ui_menu_default_genshit(void)
{
	tft180_show_chinese(8*4,  16*0,   16, (const uint8 *)font_huitui, 2, RGB565_BLUE);           		// 回退
	tft180_show_chinese(8*4,  16*4,   16, (const uint8 *)font_yuanshenniubi, 4, RGB565_BLUE);           // 原神牛逼
}
