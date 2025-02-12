#include "menu.h"
#include "key.h"
#include "oled.h"
#include "stdio.h"
#include "string.h"


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
                case 1:mode_select = (mode_select+1)%5;
                    break;
                case 2:return mode_select;
                  
                default:
                    break;
                
            }
        }
        
        sprintf((char*)g_oledstring,"g_mode:%d",mode_select);
        OLED_ShowString(16,0,g_oledstring,16,1);
        OLED_Refresh();
        switch(mode_select)
        {
            case 0:OLED_ShowString(16,16,"mode0",16,1);
                    OLED_Refresh();
                break;
            case 1:OLED_ShowString(16,16,"mode1",16,1);
                    OLED_Refresh();
                break;
            case 2:OLED_ShowString(16,16,"mode2",16,1);
                    OLED_Refresh();
                break;
            case 3:OLED_ShowString(16,16,"mode3",16,1);
                    OLED_Refresh();
                break;
            case 4:OLED_ShowString(16,16,"mode4",16,1);
                    OLED_Refresh();
                break;
        }
        
    }
}












