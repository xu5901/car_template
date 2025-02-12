#include "pid.h"


//定义全局变量
_pid g_pid_speed1,g_pid_speed2;    
_pid g_pid_location1,g_pid_location2;


/**
  * @brief  PID参数初始化
  * @note   无
  * @retval 无
  */
void pid_param_init(void)
{
    /* 电机1参数初始化 */
    /* 位置相关初始化参数 */
    g_pid_location1.target_val=0.0;
    g_pid_location1.actual_val=0.0;
    g_pid_location1.err=0.0;
    g_pid_location1.err_last=0.0;
    g_pid_location1.integral=0.0;
  
    g_pid_location1.Kp = 0.24;
    g_pid_location1.Ki = 0.0;
    g_pid_location1.Kd = 0.0;
  
    /* 速度相关初始化参数 */
    g_pid_speed1.target_val=0.0;
    g_pid_speed1.actual_val=0.0;
    g_pid_speed1.err=0.0;
    g_pid_speed1.err_last=0.0;
    g_pid_speed1.integral=0.0;
  
    g_pid_speed1.Kp = 10.0;
    g_pid_speed1.Ki = 2.4;
    g_pid_speed1.Kd = 0.0;
    
    /* 电机2参数初始化 */
    /* 位置相关初始化参数 */
    g_pid_location2.target_val=0.0;
    g_pid_location2.actual_val=0.0;
    g_pid_location2.err=0.0;
    g_pid_location2.err_last=0.0;
    g_pid_location2.integral=0.0;
  
    g_pid_location2.Kp = 0.24;
    g_pid_location2.Ki = 0.0;
    g_pid_location2.Kd = 0.0;
  
    /* 速度相关初始化参数 */
    g_pid_speed2.target_val=0.0;
    g_pid_speed2.actual_val=0.0;
    g_pid_speed2.err=0.0;
    g_pid_speed2.err_last=0.0;
    g_pid_speed2.integral=0.0;
  
    g_pid_speed2.Kp = 10.0;
    g_pid_speed2.Ki = 2.4;
    g_pid_speed2.Kd = 0.0;
        
#if defined(PID_ASSISTANT_EN)
    float pid_temp[3] = {pid.Kp, pid.Ki, pid.Kd};
    set_computer_value(SEND_P_I_D_CMD, CURVES_CH1, pid_temp, 3);     // 给通道 1 发送 P I D 值
#endif
}

/**
  * @brief  设置目标值
  * @param  pid结构体,temp_val目标值
  *	@note   无
  * @retval 无
  */
void set_pid_target(_pid *pid, float temp_val)
{
    pid->target_val = temp_val;    // 设置当前的目标值
}

/**
  * @brief  获取目标值
  * @param  pid结构体
  * @note   无
  * @retval pid目标值
  */
float get_pid_target(_pid *pid)
{
    return pid->target_val;    // 设置当前的目标值
}

/**
 * @brief       pid结构体, p,i,d值
 * @param       无
 * @retval      无
 */
void set_p_i_d(_pid *pid, float p, float i, float d)
{
    pid->Kp = p;    // 设置比例系数 P
    pid->Ki = i;    // 设置积分系数 I
    pid->Kd = d;    // 设置微分系数 D
}

/**
  * @brief  位置PID算法实现
  * @param  actual_val:实际值
  * @note   无
  * @retval 通过PID计算后的输出
  */
float location_pid_realize(_pid *pid, float actual_val)  //位置环光个Kp好像也可以
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;
  
//    /* 设定闭环死区 */   //外环死区可以不要 
//    if((pid->err >= -0.1) && (pid->err <= 0.1)) 
//    {
//      pid->err = 0;
//      pid->integral = 0;
//    }
    
    pid->integral += pid->err;    // 误差累积

    /*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
                        +pid->Ki*pid->integral
                        +pid->Kd*(pid->err-pid->err_last);
  
    /*误差传递*/
    pid->err_last=pid->err;
    
    /*返回当前实际值*/
    return pid->actual_val;
}

/**
  * @brief  速度PID算法实现
  * @param  actual_val:实际值
  * @note   无
  * @retval 通过PID计算后的输出
  */
float speed_pid_realize(_pid *pid, float actual_val)
{
    /*计算目标值与实际值的误差*/
    pid->err = pid->target_val - actual_val;

    /* 设定闭环死区 */
    /* 误差在-0.5-0.5之间时不再当做误差进行计算 */
    if((pid->err < 0.5f ) && (pid->err > -0.5f))  //差1这么多可以吗？运行1分钟，位置差为1个轮子的周长 
    {
      pid->err = 0.0f;
    }
    
    pid->integral += pid->err;    // 误差累积
    
    /*积分限幅*/
    if(pid->integral >= 10000) {pid->integral =10000;}//待确定
    else if(pid->integral < -10000) {pid->integral = -10000;}

    /*PID算法实现*/
    pid->actual_val = pid->Kp*pid->err
                        +pid->Ki*pid->integral
                        +pid->Kd*(pid->err-pid->err_last);
  
    /*误差传递*/
    pid->err_last=pid->err;
    
    /*返回当前实际值*/
    return pid->actual_val;//这个值是速度环的输出值，并不是当前电机的实际值
}


