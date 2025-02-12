#include "motor.h"
#include "tim.h"

/* 电机使能标志位 */
uint8_t g_is_motor1_en = 0;
uint8_t g_is_motor2_en = 0;

/**
 * @brief       电机装载函数，把PID运算后的最终值装载到电机
 * @param       motor1_pwm为左电机,motor1_pwm为右电机
 * @retval      无
 */
void load_motor_pwm(int motor1_pwm,int motor2_pwm)
{
    /*左电机装载*/
    //1.研究正负号,对应正反转,反向时修改此处
    if(motor1_pwm>0)  //正转
    {
        AIN1_RESET;
        AIN2_SET;
    }
    else  //反转
    {
        AIN1_SET;
        AIN2_RESET;
    }
    //2.装载PWM值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_4,find_abs(motor1_pwm));   // 设置比较寄存器的值 
    
    /*右电机装载*/
    //1.研究正负号,对应正反转,反向时修改此处
    if(motor2_pwm>0)  //正转
    {
        BIN1_RESET;
        BIN2_SET;
    }
    else  //反转
    {
        BIN1_SET;
        BIN2_RESET;
    }
    //2.装载PWM值
    __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,find_abs(motor2_pwm));   // 设置比较寄存器的值 
    
}

/**
 * @brief       电机PWM限幅函数，装载前必须限幅一下,防止过载
 * @param       *p_motor1_pwm为左电机的PWM变量指针,*p_motor2_pwm为右电机的PWM变量指针
 * @retval      无
 */
void limit_motor_pwm(int *p_motor1_pwm,int *p_motor2_pwm)
{
    if(*p_motor1_pwm > PWM_MAX)*p_motor1_pwm = PWM_MAX;
    if(*p_motor1_pwm < PWM_MIN)*p_motor1_pwm = PWM_MIN;
    
    if(*p_motor2_pwm > PWM_MAX)*p_motor2_pwm = PWM_MAX;
    if(*p_motor2_pwm < PWM_MIN)*p_motor2_pwm = PWM_MIN;
}

/**
 * @brief       求绝对值函数
 * @param       p
 * @retval      p的绝对值
 */
int find_abs(int p)
{
    int q;
    q = p>0?p:(-p);
    return q;
}


/**
  * @brief  使能电机1
  * @param  无
  * @retval 无
  */
void set_motor1_enable(void)
{
    g_is_motor1_en  = 1;
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);
}

/**
  * @brief  禁用电机1
  * @param  无
  * @retval 无
  */
void set_motor1_disable(void)
{
    g_is_motor1_en  = 0;
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_4);
}

/**
  * @brief  使能电机2
  * @param  无
  * @retval 无
  */
void set_motor2_enable(void)
{
    g_is_motor2_en  = 1;
    HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
}

/**
  * @brief  禁用电机2
  * @param  无
  * @retval 无
  */
void set_motor2_disable(void)
{
    g_is_motor2_en  = 0;
    HAL_TIM_PWM_Stop(&htim1,TIM_CHANNEL_1);
}

