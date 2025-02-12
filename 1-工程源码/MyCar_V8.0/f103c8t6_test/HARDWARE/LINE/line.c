#include "line.h"
#include "jy61p.h"

/**
 * @brief       灰度巡线补偿值获取
 * @param       无
 * @retval      巡线偏移的值
 */
int32_t line_err(void)
{
    uint8_t hw1_val, hw2_val, hw3_val, hw4_val;
    int32_t line_num;
    
    /* 读取并记录电平状态，这里根据实际01代表的情况修改，可能相反，传感器越多效果越好 */
    if(HW1==0){hw1_val = 1;} else {hw1_val = 0;}
    if(HW2==0){hw2_val = 1;} else {hw2_val = 0;}
    if(HW3==0){hw3_val = 1;} else {hw3_val = 0;}
    if(HW4==0){hw4_val = 1;} else {hw4_val = 0;}

    /* 巡线值 */
    if(hw1_val==0&&hw2_val==0&&hw3_val==0&&hw4_val==0)  line_num=0;
    if(hw1_val==1&&hw2_val==0&&hw3_val==0&&hw4_val==0)  line_num=10;
    if(hw1_val==0&&hw2_val==1&&hw3_val==0&&hw4_val==0)  line_num=20;
    if(hw1_val==0&&hw2_val==0&&hw3_val==1&&hw4_val==0)  line_num=-10;
    if(hw1_val==0&&hw2_val==0&&hw3_val==0&&hw4_val==1)  line_num=-20;
                                             
    if(hw1_val==0&&hw2_val==1&&hw3_val==1&&hw4_val==0)  line_num=0;
    if(hw1_val==1&&hw2_val==1&&hw3_val==0&&hw4_val==0)  line_num=15;
    if(hw1_val==0&&hw2_val==0&&hw3_val==1&&hw4_val==1)  line_num=-15;
    
    return line_num;
}


/**
 * @brief       角度巡线补偿值获取
 * @param       无
 * @retval      角度偏移量
 */
/* 以0度为基准 */
int32_t yaw_err0(void)
{
    int32_t yaw_num;
    if((g_yaw_jy61>0)&&(g_yaw_jy61<90)) yaw_num = g_yaw_jy61;
    else if((g_yaw_jy61>270)&&(g_yaw_jy61<360)) yaw_num = g_yaw_jy61-360;
    else yaw_num=0;
    return yaw_num;
}
/* 掉头后以180度为基准 */
int32_t yaw_err180(void)
{
    int32_t yaw_num;
    if((g_yaw_jy61>180)&&(g_yaw_jy61<270))  yaw_num = g_yaw_jy61-180;
    else if((g_yaw_jy61>90)&&(g_yaw_jy61<180))  yaw_num = g_yaw_jy61-180;
    else yaw_num=0;
    return yaw_num;
}





