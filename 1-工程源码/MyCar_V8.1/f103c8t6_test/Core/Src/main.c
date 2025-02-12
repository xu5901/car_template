/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "oled.h"
#include "bmp.h"
#include "encoder.h"
#include "motor.h"
#include "pid.h"
#include "control.h"
#include "protocol.h"
//#include "mpu6050.h"
//#include "inv_mpu.h"
//#include "inv_mpu_dmp_motion_driver.h" 
#include "jy61p.h"
#include "key.h"
#include "menu.h"
#include "line.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
uint8_t g_oledstring[50];//OLED显示
//float g_pitch_6050,g_roll_6050,g_yaw_6050; // MPU6050的俯仰角 横滚角 航向角
extern uint8_t g_usart2_receivedata;//串口2接收数据
uint8_t g_mode = 0;//小车运行模式

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);//开启定时器2 编码器模式
  HAL_TIM_Encoder_Start(&htim4,TIM_CHANNEL_ALL);//开启定时器4 编码器模式
  HAL_TIM_Base_Start_IT(&htim2);                //开启定时器2 中断
  HAL_TIM_Base_Start_IT(&htim4);                //开启定时器4 中断
  
  set_motor1_enable();//使能电机
  set_motor2_enable();//使能电机
  
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_4);//开启定时器1 通道4 PWM输出
  HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);//开启定时器1 通道1 PWM输出

  HAL_TIM_Base_Start_IT(&htim3); //开启定时器3中断
   
  pid_param_init();//pid参数初始化
   
//  __HAL_UART_ENABLE_IT(&huart1,UART_IT_RXNE);    //串口1接收中断，只开启接收中断，与上位机通信
  HAL_UART_Receive_IT(&huart2,&g_usart2_receivedata,1);  //串口2接收数据中断
  
//  protocol_init();       //这个初始化了才能通过上位机控制呀, 再看这个注意事项
     
  OLED_Init(); //OLED初始化
  OLED_ColorTurn(0);//0正常显示，1 反色显示
  OLED_DisplayTurn(0);//0正常显示 1 屏幕翻转显示
  
  /* MPU6050初始化 */
//  HAL_Delay(500);//延时0.5秒 6050上电稳定后初始化
//  MPU_Init(); //初始化MPU6050
//  while(MPU_Init()!=0);
//  while(mpu_dmp_init()!=0);
  

    uint8_t key = 0;
    uint16_t target_angle = 0;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//      receiving_process();//解析野火上位机通信协议,调试电机位置速度串级PID时用,不用时注释
      
      
      /* 菜单测试 */
      if(g_mode == 0)   //启动菜单选择
      {
          g_mode = menu_start_select();
      }
      else if(g_mode == 1)//巡线直行测试
      {
          g_Line_Flag = 1;
      }
      else if(g_mode == 2)//位置环测试
      {
          key = key_scan(0);
           if (key)
            {
                switch (key)
                {
                    case KEY0_PRES:
                        HAL_Delay(1000);//给出时间拿开手，防止干扰车子启动
                         car_go(120);
                        break;
                    case KEY1_PRES:
                        HAL_Delay(1000);//给出时间拿开手，防止干扰车子启动
                        car_go(60);
                        break;
                }
            }
      }
      else if(g_mode == 3)//位置环转弯测试
      {
           key = key_scan(0);
           if (key)
            {
                switch (key)
                {
                    case KEY0_PRES:
                        HAL_Delay(500);//给出时间拿开手，防止干扰车子启动
                        car_spin(left_90);
                        break;
                    case KEY1_PRES:
                        HAL_Delay(500);//给出时间拿开手，防止干扰车子启动
                        car_spin(back_180);
                        break;
                }
            }
      }
      else if(g_mode == 4)//角度环测试
      {
          g_Angle_Flag = 1;
          key = key_scan(0);
           if (key)
            {
                switch (key)
                {
                    case KEY0_PRES:
                        HAL_Delay(500);//给出时间拿开手，防止干扰车子启动
                        target_angle = target_angle+90;
                        if(target_angle>360)target_angle = 0;
                        set_pid_target(&g_pid_turn_angle,target_angle);
                        break;
                    case KEY1_PRES:
                        HAL_Delay(500);//给出时间拿开手，防止干扰车子启动
                        target_angle = target_angle+180;
                        if(target_angle>360)target_angle = 0;
                        set_pid_target(&g_pid_turn_angle,target_angle);
                        break;
                }
            }
      }
     sprintf((char*)g_oledstring,"Yaw:%6.2f",g_yaw_jy61);//显示
     OLED_ShowString(0,32,g_oledstring,16,1);//这个是oled驱动里面的，是显示位置的一个函数，
     OLED_Refresh();
      
      
      
//      /* 查看jy901角度测试 */
//      sprintf((char*)g_oledstring,"Roll:%6.2f",g_roll_jy61);//显示
//      OLED_ShowString(0,0,g_oledstring,16,1);//这个是oled驱动里面的，是显示位置的一个函数，
//      sprintf((char*)g_oledstring,"Pitch:%6.2f",g_pitch_jy61);//显示
//      OLED_ShowString(0,16,g_oledstring,16,1);//这个是oled驱动里面的，是显示位置的一个函数，
//      sprintf((char*)g_oledstring,"Yaw:%6.2f",g_yaw_jy61);//显示
//      OLED_ShowString(0,32,g_oledstring,16,1);//这个是oled驱动里面的，是显示位置的一个函数，
//      OLED_Refresh();
      
      
//      /* 查看mpu6050角度测试 */
//      sprintf((char *)g_oledstring,"pitch:%.2f",g_pitch_6050);//显示6050数据
//      OLED_ShowString(0,0,g_oledstring,8,1);
//      sprintf((char *)g_oledstring,"roll:%.2f",g_roll_6050);//显示6050数据
//      OLED_ShowString(0,8,g_oledstring,8,1);
//      sprintf((char *)g_oledstring,"yaw:%.2f",g_yaw_6050);//显示6050数据
//      OLED_ShowString(0,16,g_oledstring,8,1);
//      OLED_Refresh();
//       //mpu_dmp_get_data(&pitch,&roll,&yaw);//返回值:0,DMP成功解出欧拉角
//      while(mpu_dmp_get_data(&g_pitch_6050,&g_roll_6050,&g_yaw_6050)!=0){}  //这个可以解决经常读不出数据的问题
      
      
//          /* 查看里程数测试 */
//      sprintf((char *)g_oledstring,"motor1:%d",g_sigma_motor1pluse);//显示单位时间脉冲数
//      OLED_ShowString(0,0,g_oledstring,8,1);  //显示在OLED上
//      sprintf((char *)g_oledstring,"jour1:%.2f",g_motor1_journey_cm);//显示总里程数
//      OLED_ShowString(0,8,g_oledstring,8,1);  //显示在OLED上
//      
//      sprintf((char *)g_oledstring,"motor2:%d",g_sigma_motor2pluse);//显示单位时间脉冲数
//      OLED_ShowString(0,16,g_oledstring,8,1);  //显示在OLED上
//      sprintf((char *)g_oledstring,"jour2:%.2f",g_motor2_journey_cm);//显示总里程数
//      OLED_ShowString(0,24,g_oledstring,8,1);  //显示在OLED上
//      
//      OLED_Refresh();
      
      

        
//      HAL_GPIO_TogglePin(LED0_GPIO_Port,LED0_Pin);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
