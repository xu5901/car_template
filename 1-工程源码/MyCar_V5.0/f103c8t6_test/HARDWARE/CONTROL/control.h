#ifndef  __CONTROL_H
#define  __CONTROL_H

#include  "main.h"

/* 小车物理特性 */
/* 编码器物理分辨率 */
#define ENCODER_RESOLUTION  13      //编码器线数

/* 经过倍频之后的总分辨率 */
#define ENCODER_TOTAL_RESOLUTION    (ENCODER_RESOLUTION * 4)  /* 4倍频后的总分辨率 */

/* 减速电机减速比 */
#define REDUCTION_RATIO  34

/*PID计算的周期，以毫秒为单位*/
#define SPEED_PID_PERIOD  20    //这个要看定时器3的中断周期

/* 小车速度限制 */
#define TARGET_SPEED_MAX  160  //单位160rpm，1s能跑的最大距离53cm

/* 轮胎直径cm */
#define WHEEL_D  6.5

/* 小车两轮距离 */
#define LUN_JU  19   //单位cm,实际测量为准

/* 小车轮胎到车头距离 */
#define HEAD_TO_WHEEL 6    //单位cm,实际测量为准

/* 外部变量声明 */
extern int g_motor1_pwm;     //电机1装载pwm值
extern int g_motor2_pwm;     //电机2装载pwm值

extern long g_sigma_motor1pluse;       //电机1累计脉冲
extern long g_sigma_motor2pluse;       //电机2累计脉冲
extern short g_unittime_motor1pluse;   //电机1单位时间脉冲
extern short g_unittime_motor2pluse;   //电机2单位时间脉冲
extern float g_motor1_journey_cm;    //电机1走过的路程
extern float g_motor2_journey_cm;    //电机2走过的路程

extern uint8_t g_Line_Flag;    //巡线标志位,0不巡线,1巡线
extern uint8_t g_Stop_Flag;    //停止标志位,0行驶,1停止
extern uint8_t g_Spin_Start_Flag;    //转向开始标志位
extern uint8_t g_Spin_Succeed_Flag;  //转向结束标志位
extern uint8_t g_Turn_Flag;           //转向标志位

/* 小车旋转选项 */
typedef enum
{
    left_90,
    right_90,
    back_180
}spin_dir;

/* 小车控制函数 */
void car_go(int32_t distance_cm);
void car_spin(spin_dir direction);

/* pid实现函数 */
float speed1_pid_control(void);
float location1_pid_control(void);
float speed2_pid_control(void);
float location2_pid_control(void);
void location_speed_control(void);

#endif
