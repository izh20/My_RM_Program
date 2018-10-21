#include "robomaster_task.h"


/**
  * @brief  TIM2中断处理函数主要运行的程序，5MS执行一次
  * @param  无
  * @retval 
  * @usage  飞控主要程序执行 
  *               
  */

	void Test_Period(Testime *Time_Lab)
{
   Time_Lab->Last_Time=Time_Lab->Now_Time;
   Time_Lab->Now_Time=(10000*TIME_ISR_CNT+TIM2->CNT)/1000.0;//单位ms
   Time_Lab->Time_Delta=Time_Lab->Now_Time-Time_Lab->Last_Time;
   Time_Lab->Time_Delta_INT=(uint16_t)(Time_Lab->Time_Delta);
}

Testime Time1_Delta;	
int	cnt=0;
void task() 
{	
	cnt++;
	if(cnt==100)    //0.5s进入一次，使第7个等以2HZ频率闪烁，从而判断中断程序正常运行
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
	}
	Test_Period(&Time1_Delta);//系统调度时间测试器
	//mpu_get_data();//获得imu原始数据
	//imu_ahrs_update();//更新四元数和imu姿态
	GET_MPU_DATA();

	
}
