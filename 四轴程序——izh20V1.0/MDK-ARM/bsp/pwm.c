/**
  ******************************************************************************
  * @file		 pwm.c
  * @author  izh20
  * @version V1.0.0
  * @date    2018/10/20
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
#include "pwm.h"

void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty)
	{
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (10000*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (10000*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (10000*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (10000*duty) - 1;break;
	}
	
}
void init_TIM5_PWM()
{
	PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.10);
	PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.10);
	PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.10);
	PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.10);
}



