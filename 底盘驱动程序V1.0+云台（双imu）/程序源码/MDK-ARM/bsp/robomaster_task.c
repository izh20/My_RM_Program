#include "robomaster_task.h"


/**
  * @brief  TIM2中断处理函数主要运行的程序，5MS执行一次
  * @param  无
  * @retval 
  * @usage  用于底盘的驱动，结合imu进行底盘控制，后期可用来走猫步        
  *               
  */
int	cnt=0, //判断中断程序是否正常
		cnt1=0,//判断底盘控制是否正常
		cnt_yuntai;//判断云台控制是否正常
void task() 
{	
	cnt++;
	cnt1++;
	cnt_yuntai++;
	if(cnt==100)    //0.5s进入一次，使第7个等以2HZ频率闪烁，从而判断中断程序正常运行
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
	}
	MPU9250_GET_DATA();//获得外置mpu9250imu原始数据
	mpu_get_data();//获得板载imu原始数据
	imu_ahrs_update();//更新四元数和imu姿态
	imu_attitude_update();//解算imu的pitch,roll,yaw角度
	chassis_control();		//底盘电机的控制
	pan_tilt_control();
	shoot_control();
	
}
