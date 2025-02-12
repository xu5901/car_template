#ifndef __LINE_H
#define __LINE_H

#include  "main.h"

#define HW1   HAL_GPIO_ReadPin(HW1_GPIO_Port,HW1_Pin) //读取红外对管连接的GPIO电平
#define HW2   HAL_GPIO_ReadPin(HW2_GPIO_Port,HW2_Pin)
#define HW3   HAL_GPIO_ReadPin(HW3_GPIO_Port,HW3_Pin)
#define HW4   HAL_GPIO_ReadPin(HW4_GPIO_Port,HW4_Pin)



int32_t line_err(void);
int32_t yaw_err0(void);
int32_t yaw_err180(void);

#endif
