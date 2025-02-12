#ifndef  __MOTOR_H
#define  __MOTOR_H

#include  "main.h"

/*电机方向引脚控制宏定义*/
#define AIN1_SET    HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_SET)
#define AIN1_RESET  HAL_GPIO_WritePin(AIN1_GPIO_Port,AIN1_Pin,GPIO_PIN_RESET)
#define AIN2_SET    HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_SET)
#define AIN2_RESET  HAL_GPIO_WritePin(AIN2_GPIO_Port,AIN2_Pin,GPIO_PIN_RESET)

#define BIN1_SET    HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_SET)
#define BIN1_RESET  HAL_GPIO_WritePin(BIN1_GPIO_Port,BIN1_Pin,GPIO_PIN_RESET)
#define BIN2_SET    HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_SET)
#define BIN2_RESET  HAL_GPIO_WritePin(BIN2_GPIO_Port,BIN2_Pin,GPIO_PIN_RESET)

/*电机PWM限幅宏定义,ARR为7200*/
#define  PWM_MAX   7000   /*正方向最大速度*/
#define  PWM_MIN  -7000   /*负方向最大速度*/

/*电机使能标志位*/
extern uint8_t g_is_motor1_en;
extern uint8_t g_is_motor2_en;

/*电机控制函数*/
void load_motor_pwm(int motor1_pwm,int motor2_pwm);
void limit_motor_pwm(int *p_motor1_pwm,int *p_motor2_pwm);
int find_abs(int p);
void set_motor1_enable(void);
void set_motor1_disable(void);
void set_motor2_enable(void);
void set_motor2_disable(void);

#endif


