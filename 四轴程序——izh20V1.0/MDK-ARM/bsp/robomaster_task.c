#include "robomaster_task.h"


/**
  * @brief  TIM2�жϴ�������Ҫ���еĳ���5MSִ��һ��
  * @param  ��
  * @retval 
  * @usage  �ɿ���Ҫ����ִ�� 
  *               
  */

	void Test_Period(Testime *Time_Lab)
{
   Time_Lab->Last_Time=Time_Lab->Now_Time;
   Time_Lab->Now_Time=(10000*TIME_ISR_CNT+TIM2->CNT)/1000.0;//��λms
   Time_Lab->Time_Delta=Time_Lab->Now_Time-Time_Lab->Last_Time;
   Time_Lab->Time_Delta_INT=(uint16_t)(Time_Lab->Time_Delta);
}

Testime Time1_Delta;	
int	cnt=0;
void task() 
{	
	cnt++;
	if(cnt==100)    //0.5s����һ�Σ�ʹ��7������2HZƵ����˸���Ӷ��ж��жϳ�����������
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
	}
	Test_Period(&Time1_Delta);//ϵͳ����ʱ�������
	//mpu_get_data();//���imuԭʼ����
	//imu_ahrs_update();//������Ԫ����imu��̬
	GET_MPU_DATA();

	
}
