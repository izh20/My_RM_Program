#include "robomaster_task.h"


/**
  * @brief  TIM2�жϴ�������Ҫ���еĳ���5MSִ��һ��
  * @param  ��
  * @retval 
  * @usage  ���ڵ��̵����������imu���е��̿��ƣ����ڿ�������è��        
  *               
  */
int	cnt=0,cnt1=0;
void task() 
{	
	cnt++;
	cnt1++;
	if(cnt==100)    //0.5s����һ�Σ�ʹ��7������2HZƵ����˸���Ӷ��ж��жϳ�����������
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
	}
	
	mpu_get_data();//���imuԭʼ����
	imu_ahrs_update();//������Ԫ����imu��̬
	imu_attitude_update();//����imu��pitch,roll,yaw�Ƕ�
	chassis_control();		//���̵���Ŀ���
	shoot_control();			//Ħ�����Լ���������Ŀ���
	
}
