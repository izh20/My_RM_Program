/**
  ******************************************************************************
  * @file		 pwm.h
  * @author  izh20
  * @version V1.0.0
  * @date    2018/10/20
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef _PWM_H
#define _PWM_H
#include "robomaster_common.h"

void init_TIM5_PWM(void);
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);

#endif
