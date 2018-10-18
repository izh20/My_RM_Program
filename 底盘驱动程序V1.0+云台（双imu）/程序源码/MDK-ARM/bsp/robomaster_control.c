/**
  *@file robomaster_control.c
  *@date 2018-10-5
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"


void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty)
	{
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (10000*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (10000*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (10000*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (10000*duty) - 1;break;
	}
	
}

void shoot_control()
{
	if(remote_control.switch_left!=3)
		{
				if(remote_control.switch_right==3)
				{
//				  motor_pid[6].target=4000;
//					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);//速度环
//					set_bodan_current(&hcan1,motor_pid[6].output);
					set_rammer_current(&hcan1,0);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.14);
				}
				if(remote_control.switch_right==2)
				{	
					motor_pid[6].target=7000;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					
					set_rammer_current(&hcan1,motor_pid[6].output);
				}
				if(remote_control.switch_right==1)
				{
					set_rammer_current(&hcan1,0);
					init_TIM5_PWM();
				}

		}else
		{
			set_rammer_current(&hcan1,0);
			init_TIM5_PWM();
		}
}


extern int16_t moto_ctr[6];
int32_t set_spd = 0;//速度参数
int32_t turn=0;     //转弯
extern int cnt1;
void chassis_control()
{
	if(remote_control.switch_left!=3)
	{
	if(cnt1==100)//0.5s进入一次，使第4个led以2HZ频率闪烁，判断底盘程序正常运行
		{
				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_4);
				cnt1=0;
		}
	DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
	motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
	motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。
	motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
	motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
	
	set_moto_current(&hcan1,motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
														motor_pid[1].output,
														motor_pid[2].output,
														motor_pid[3].output);
	
	
	}
	else
		set_moto_current(&hcan1,0,0,0,0);
}



extern int cnt_yuntai;
#ifdef INFANTRY_YUNTAI
/**
  * @brief  上电云台位置初始化（回中）
  * @param  无
  * @retval 
  * @usage  用于底盘的驱动，结合imu进行底盘控制，后期可用来走猫步       后期更新  
  *               
  */
void pan_tilt_position_init()
{
	
}

void pan_tilt_control()
{
	if(remote_control.switch_left!=3)
	{
		if(cnt_yuntai==100)//0.5s进入一次，使第1个led以2HZ频率闪烁，判断底盘程序正常运行
		{
				HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
				cnt_yuntai=0;
		}
		/********************步兵相对于底盘控制，反馈信号为电机的机械角度***********************/
		motor_pid[4].target=remote_control.ch2*1000/660+3645;//修正     pitch轴    步兵云台为yaw
		motor_pid[4].f_cal_pid(&motor_pid[4],moto_chassis[4].angle);//机械角度环
		
		motor_pid[5].target=remote_control.ch4+4900;//修正     yaw轴       步兵云台为pitch
		motor_pid[5].f_cal_pid(&motor_pid[5],moto_chassis[5].angle);//机械角度环
		pid_pithch_speed.target=motor_pid[5].output;
		motor_pid[5].f_cal_pid(&pid_pithch_speed,imu.gy);//角速度环
		set_pan_tilt_current(&hcan1,motor_pid[4].output,pid_pithch_speed.output);
		
	}
	else
		set_pan_tilt_current(&hcan1,0,0);
}


#endif
