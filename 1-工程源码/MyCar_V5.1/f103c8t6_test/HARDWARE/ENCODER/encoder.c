#include "encoder.h"
#include "tim.h"


 /**
  * @brief       脉冲读取函数
  * @param       *htim2左电机,*htim4右电机
  * @retval      对应定时器读取到的脉冲值
  */
int read_pluse(TIM_HandleTypeDef *htim)
{
    int pluse;
    pluse = (short)__HAL_TIM_GET_COUNTER(htim); //获取计数值
    __HAL_TIM_SET_COUNTER(htim,0); //清零计数值
    return pluse;
}
