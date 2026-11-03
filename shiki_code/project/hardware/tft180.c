#include "tft180.h"

void my_tft180_init(tft180_dir_enum disp_dir, tft180_font_size_enum disp_font_size, rgb565_color_enum disp_font_color, rgb565_color_enum disp_background_color)
{
	tft180_set_dir(disp_dir);										// 设置tft180显示画面方向
    tft180_set_font(disp_font_size);								// 设置tft180显示字体大小				
    tft180_set_color(disp_font_color, disp_background_color);		// 设置tft180显示字体颜色、字体的背景颜色
    tft180_init();													// 设置tft180SPI初始化
	tft180_clear();	
}




