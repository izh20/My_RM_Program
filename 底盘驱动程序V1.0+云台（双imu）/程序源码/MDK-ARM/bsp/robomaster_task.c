#include "robomaster_task.h"


/**
  * @brief  TIM2�жϴ�������Ҫ���еĳ���5MSִ��һ��
  * @param  ��
  * @retval 
  * @usage  ���ڵ��̵����������imu���е��̿��ƣ����ڿ�������è��        
  *               
  */
int	cnt=0, //�ж��жϳ����Ƿ�����
		cnt1=0,//�жϵ��̿����Ƿ�����
		cnt_yuntai;//�ж���̨�����Ƿ�����
void task() 
{	
	cnt++;
	cnt1++;
	cnt_yuntai++;
	if(cnt==100)    //0.5s����һ�Σ�ʹ��7������2HZƵ����˸���Ӷ��ж��жϳ�����������
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_1);
	}
	MPU9250_GET_DATA();//�������mpu9250imuԭʼ����
	mpu_get_data();//��ð���imuԭʼ����
	imu_ahrs_update();//������Ԫ����imu��̬
	imu_attitude_update();//����imu��pitch,roll,yaw�Ƕ�
	chassis_control();		//���̵���Ŀ���
	pan_tilt_control();
	shoot_control();
	
}
