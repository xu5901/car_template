#include "control.h"
#include "tim.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"



void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim == &htim3)//20ms定时器中断进行PID计算
    {
        
        
        
    }
    
}

