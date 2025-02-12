#include "menu.h"
#include "key.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"
#include "jy61p.h"

extern uint8_t g_oledstring[];

/**
 * @brief       启动菜单选择
 * @param       无
 * @retval      modeselect模式选择
 */
uint8_t menu_start_select(void)
{
    uint8_t mode_select = 0;
    uint8_t key_num = 0;

    while(1)
    {
        key_num = key_scan(0);
        if(key_num)
        {
            switch(key_num)
            {
                case 1:mode_select = (mode_select+1)%5; //模式切换
                    break;
                case 2:HAL_Delay(2000); //这个延时是给时间把手拿开，防止对车子启动造成干扰
                    return mode_select; //选择好模式，退出循环了
                  
                default:
                    break;
                
            }
        }
        
        sprintf((char*)g_oledstring,"g_mode:%d",mode_select);
        OLED_ShowString(16,0,g_oledstring,16,1);
        OLED_Refresh();
        switch(mode_select)
        {
            case 0:OLED_ShowString(16,16,"select       ",16,1);
                    OLED_Refresh();
                break;
            case 1:OLED_ShowString(16,16,"line_test    ",16,1);
                    OLED_Refresh();
                break;
            case 2:OLED_ShowString(16,16,"car_go_test  ",16,1);
                    OLED_Refresh();
                break;
            case 3:OLED_ShowString(16,16,"car_spin_test",16,1);
                    OLED_Refresh();
                break;
            case 4:OLED_ShowString(16,16,"turn_test    ",16,1);
                    OLED_Refresh();
                break;
        }
        sprintf((char*)g_oledstring,"Yaw:%6.2f",g_yaw_jy61);//显示
        OLED_ShowString(0,32,g_oledstring,16,1);//这个是oled驱动里面的，是显示位置的一个函数，
        OLED_Refresh();
        
    }
}












