/**
  ******************************************************************************
  * File Name          : main.c
	* author						 : 周恒
	* Date							 :2018.10.07
  * Description        ;while循环中只运行山外的上位机程序，姿态解算，底盘电机的驱动和摩擦轮，拨弹电机的控制全在robomaster_task.c文件中运行
												目前程序只进行了底盘电机的速度闭环，拨弹电机的速度闭环，云台电机还未进行控制，下一步可将角度信息融合到底盘的控制中去
	
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* USER CODE BEGIN Includes */
#include "robomaster_common.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
PID_TypeDef motor_pid[7];//初始化7个电机的pid结构体 其中0 1 2 3对应底盘电机，4 5 对应云台电机 6对应拨弹电机，目前只用到了0 1 2 3 6号电机

uint16_t TIM_COUNT[2];
#define SpeedStep 500

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_CAN1_Init();//can通信
  MX_USART1_UART_Init();//遥控器接收，串口中断
  MX_TIM1_Init();//用来计时，测试程序运行的时间
  MX_TIM5_Init();//产生4路pwm波来控制摩擦轮
  MX_USART6_UART_Init();//用于上位机调试
  MX_SPI5_Init();//用于imu驱动
  MX_TIM2_Init();//中断控制程序  主要程序在TIM2中运行
  MX_TIM3_Init();//未用
  MX_TIM4_Init();//未用
  MX_USART2_UART_Init();//蓝牙串口  未用
  MX_USART3_UART_Init();//大疆SDK串口 未用

  /* USER CODE BEGIN 2 */
	/**TIM5 GPIO Configuration    
	PI0     ------> TIM5_CH4
	PH12     ------> TIM5_CH3
	PH11     ------> TIM5_CH2
	PH10     ------> TIM5_CH1 
	*/
my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //启动串口接收
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
  

	
  init_TIM5_PWM();			//初始化TIM5
	mpu_device_init();   //在初始化imu的时候，要先初始化SPI5和GPIOF6 不然无法初始化imu  该imu为板载imu
	init_quaternion();	//初始化四元数，用于姿态解算
	all_pid_init();			//所有电机的pid初始化
  //MPU9250_Init();  																	
  Butterworth_Parameter_Init();//滤波器参数初始化
	
	
	
	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	HAL_TIM_Base_Start_IT(&htim2);//开启定时器2  程序主要任务都在定时器2中断里完成呢
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
//		wave_form_data[0] =motor_pid[6].output;
//	  wave_form_data[1] =motor_pid[6].target;
//	  wave_form_data[2] =moto_chassis[6].speed_rpm;
//	  wave_form_data[3] =moto_chassis[6].angle;
//	  wave_form_data[4] =moto_chassis[6].total_angle;//moto_chassis[0].real_current;
//	  wave_form_data[5] =moto_chassis[6].real_current;//moto_chassis[0].hall;
		
//pitch轴云台电机		
//		wave_form_data[0] =pid_pithch_speed.output;
//	  wave_form_data[1] =motor_pid[5].target;
//	  wave_form_data[2] =pid_pithch_speed.target;
//	  wave_form_data[3] =moto_chassis[5].angle;    //步兵云台为pitch   
//	  wave_form_data[4] =imu.gy;
//	  wave_form_data[5] =remote_control.ch4;   //步兵云台为yaw

//外置mpu9250
//		wave_form_data[0] =(short)imu.gx;
//	  wave_form_data[1] =(short)imu_9250.wy;
//	  wave_form_data[2] =(short)imu_9250.wz;
//	  wave_form_data[3] =(short)imu_9250.ax;
//	  wave_form_data[4] =(short)imu_9250.ay;
//	  wave_form_data[5] =(short)imu.wy;

		wave_form_data[0] =(short)imu.ax;
	  wave_form_data[1] =(short)imu.ay;
	  wave_form_data[2] =(short)imu.az;
	  wave_form_data[3] =(short)imu.mx;
	  wave_form_data[4] =(short)imu.my;
	  wave_form_data[5] =(short)imu.mz;
		shanwai_send_wave_form();   //将数据传输到三外上位机，可以看到实时波形
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
