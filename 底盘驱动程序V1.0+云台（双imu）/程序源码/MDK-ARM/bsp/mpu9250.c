#include "mpu9250_reg.h"

short gyro_offsetx=0,gyro_offsety=0,gyro_offsetz=0;
float tmp1,tmp2,tmp3;
float magoffsetx=1.31454428611172,magoffsety=-1.21753632395713,magoffsetz=1.6567777185719;
float B[6]={0.980358187761106,-0.0105514731414606,0.00754899338354401,0.950648704823113,-0.0354995317649016,1.07449478456729};
float accoffsetx=-0.0118443148713821,accoffsety=-0.00939616830387326,accoffsetz=-0.0507891438815726;
float accsensx=1.00851297697413,accsensy=0.991366909333871,accsensz=1.00019364448499;

#define filter_high 0.8
#define filter_low 	0.2
short accoldx,accoldy,accoldz;
short magoldx,magoldy,magoldz;
short gyrooldx,gyrooldy,gyrooldz;

short igx,igy,igz;
short iax,iay,iaz;
short imx,imy,imz;
//float gx,gy,gz;
//float ax,ay,az;
//float mx,my,mz;



u8 MPU9250_Init(void)
{
    u8 res=0;
    IIC_Init();     //��ʼ��IIC����
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//��λMPU9250
    delay_ms(100);  //��ʱ100ms
	HAL_GPIO_WritePin(LED1_GPIO_Port, GPIO_PIN_14, GPIO_PIN_RESET);
		
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//����MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//�����Ǵ�����,��2000dps
	MPU_Set_Accel_Fsr(0);					       	 	//���ٶȴ�����,��2g
    MPU_Set_Rate(200);						       	 	//���ò�����200Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //�ر������ж�
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C��ģʽ�ر�
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//�ر�FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT���ŵ͵�ƽ��Ч������bypassģʽ������ֱ�Ӷ�ȡ������
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //��ȡMPU6500��ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) //����ID��ȷ
    {
			
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//����CLKSEL,PLL X��Ϊ�ο�
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//���ٶ��������Ƕ�����
		MPU_Set_Rate(200);						       	//���ò�����Ϊ200Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//��ȡAK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//��λAK8963
		delay_ms(50);
			
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//����AK8963Ϊ���β���
			
    }else return 1;
		
		calibrate();

    return 0;
}

void MPU9250_GET_DATA()
{
	MPU_Get_Gyro(&igx,&igy,&igz,&imu_9250.wx,&imu_9250.wy,&imu_9250.wz);
	MPU_Get_Accel(&iax,&iay,&iaz,&imu_9250.ax,&imu_9250.ay,&imu_9250.az);
	MPU_Get_Mag(&imx,&imy,&imz,&imu_9250.mx,&imu_9250.my,&imu_9250.mz);
}





//����MPU9250�����Ǵ����������̷�Χ
//fsr:0,��250dps;1,��500dps;2,��1000dps;3,��2000dps
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//���������������̷�Χ  
}
//����MPU9250���ٶȴ����������̷�Χ
//fsr:0,��2g;1,��4g;2,��8g;3,��16g
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//���ü��ٶȴ����������̷�Χ  
}

//����MPU9250�����ֵ�ͨ�˲���
//lpf:���ֵ�ͨ�˲�Ƶ��(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//�������ֵ�ͨ�˲���  
}

//����MPU9250�Ĳ�����(�ٶ�Fs=1KHz)
//rate:4~1000(Hz)
//����ֵ:0,���óɹ�
//    ����,����ʧ�� 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//�������ֵ�ͨ�˲���
 	return MPU_Set_LPF(rate/2);	//�Զ�����LPFΪ�����ʵ�һ��
}

/*/////////////////////////////////////////////
*@���ܣ�����������Ư��
*
*
/*////////////////////////////////////////////*/
void calibrate(void)
{
	u8 t;
	short gx,gy,gz,sumx=0,sumy=0,sumz=0;
	for (t=0;t<100;t++)
	{
		MPU_Get_Gyroscope(&gx,&gy,&gz);
		sumx=sumx+gx;
		sumy=sumy+gy;
		sumz=sumz+gz;
	}
	gyro_offsetx=-sumx/100;
	gyro_offsety=-sumy/100;
	gyro_offsetz=-sumz/100;
	

}



//�õ��¶�ֵ
//����ֵ:�¶�ֵ(������100��)
short MPU_Get_Temperature(void)
{
    u8 buf[2]; 
    short raw;
	float temp;
	MPU_Read_Len(MPU9250_ADDR,MPU_TEMP_OUTH_REG,2,buf); 
    raw=((u16)buf[0]<<8)|buf[1];  
    temp=21+((double)raw)/333.87;  
    return temp*100;;
}
/*//////////////////////////////////////////////////
////////////////////////////////*/
//�õ�������ֵ(ԭʼֵ)����Դ����ƽ��ֵ�˲��������������Ƿ�λ
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Gyroscope(short *gy,short *gx,short *gz)
{
    u8 buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR,MPU_GYRO_XOUTH_REG,6,buf);
	if(res==0)
	{
		*gy=(((u16)buf[0]<<8)|buf[1])+gyro_offsety;  
		*gx=(((u16)buf[2]<<8)|buf[3])+gyro_offsetx;  
		*gz=(((u16)buf[4]<<8)|buf[5])+gyro_offsetz;
		*gx=-*gx;
		*gx=(short)(gyrooldx*0.5+*gx*0.5);
		*gy=(short)(gyrooldy*0.5+*gy*0.5);
		*gz=(short)(gyrooldz*0.5+*gz*0.5);
		gyrooldx=*gx;
		gyrooldy=*gy;
		gyrooldz=*gz;
		
	} 	
    return res;
}
/*//////////////////////////////////////////////////
*@���ܣ�������������ݣ���λ����ÿ��
*
*
/*//////////////////////////////////////////////////*/
u8 MPU_Get_Gyro(short *igx,short *igy,short *igz,float *gx,float *gy,float *gz)
{
	u8 res;
	res=MPU_Get_Gyroscope(igx,igy,igz);
	if (res==0)
	{
	*gx=(float)(*igx)*gryo_scale;
	*gy=(float)(*igy)*gryo_scale;
	*gz=(float)(*igz)*gryo_scale;
	}
	return res;
}

//�õ����ٶ�ֵ(ԭʼֵ)����ͨ�˲����������ٶȼƷ�λ
//gx,gy,gz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Accelerometer(short *ay,short *ax,short *az)
{
    u8 buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR,MPU_ACCEL_XOUTH_REG,6,buf);
	if(res==0)
	{
		*ay=(((u16)buf[0]<<8)|buf[1]);  
		*ax=(((u16)buf[2]<<8)|buf[3]);  
		*az=(((u16)buf[4]<<8)|buf[5]);
		*ax=-*ax;
		*ax=(short)(accoldx*filter_high+*ax*filter_low);
		*ay=(short)(accoldy*filter_high+*ay*filter_low);
		*az=(short)(accoldz*filter_high+*az*filter_low);
		accoldx=*ax;
		accoldy=*ay;
		accoldz=*az;
		
	} 	
    return res;
}

/*//////////////////////////////////////////////////
*@���ܣ���ü��ٶȼ����ݣ���λg�����Լ��ٶȼƽ��в���
*
*
/*//////////////////////////////////////////////////*/
u8 MPU_Get_Accel(short *iax,short *iay,short *iaz,int16_t *ax,int16_t *ay,int16_t*az)
{
	u8 res;
	res=MPU_Get_Accelerometer(iax,iay,iaz);
	if (res==0)
	{
	tmp1=(float)(*iax)*accel_scale-accoffsetx;
	tmp2=(float)(*iay)*accel_scale-accoffsety;
	tmp3=(float)(*iaz)*accel_scale-accoffsetz;
	*ax=tmp1*accsensx;
	*ay=tmp2*accsensy;
	*az=tmp3*accsensz;
	}
	return res;
}
//�õ�������ֵ(ԭʼֵ)��ƽ���˲���������λ
//mx,my,mz:������x,y,z���ԭʼ����(������)
//����ֵ:0,�ɹ�
//    ����,�������
u8 MPU_Get_Magnetometer(short *mx,short *my,short *mz)
{
    u8 buf[6],res;  
 	res=MPU_Read_Len(AK8963_ADDR,MAG_XOUT_L,6,buf);
	if(res==0)
	{
		*mx=((u16)buf[1]<<8)|buf[0];  
		*my=((u16)buf[3]<<8)|buf[2];  
		*mz=((u16)buf[5]<<8)|buf[4];
		*my=-*my;
		*mz=-*mz;
		*mx=(short)(magoldx*0.5+*mx*0.5);
		*my=(short)(magoldy*0.5+*my*0.5);
		*mz=(short)(magoldz*0.5+*mz*0.5);
		magoldx=*mx;
		magoldy=*my;
		magoldz=*mz;
	} 	 
	MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963ÿ�ζ����Ժ���Ҫ��������Ϊ���β���ģʽ
    return res;
}

/*//////////////////////////////////////////////////
*@���ܣ���ô��������ݣ���λ��˹�����Դ����ƽ��в���
*
*
///////////////////////////////////////////////////*/
u8 MPU_Get_Mag(short *imx,short *imy,short *imz,int16_t *mx,int16_t *my,int16_t *mz)
{
	u8 res;
	res=MPU_Get_Magnetometer(imx,imy,imz);
	if (res==0)
	{
	tmp1=(float)(*imx)*mag_scale-magoffsetx;
	tmp2=(float)(*imy)*mag_scale-magoffsety;
	tmp3=(float)(*imz)*mag_scale-magoffsetz;
	*mx=B[0]*tmp1+B[1]*tmp2+B[2]*tmp3;
	*my=B[1]*tmp1+B[3]*tmp2+B[4]*tmp3;
	*mz=B[2]*tmp1+B[4]*tmp2+B[5]*tmp3;
	}
	return res;
}

//IIC����д
//addr:������ַ 
//reg:�Ĵ�����ַ
//len:д�볤��
//buf:������
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //��������
        if(IIC_Wait_Ack())      //�ȴ�ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC������
//addr:������ַ
//reg:Ҫ��ȡ�ļĴ�����ַ
//len:Ҫ��ȡ�ĳ���
//buf:��ȡ�������ݴ洢��
//����ֵ:0,����
//    ����,�������
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//������,����nACK 
		else *buf=IIC_Read_Byte(1);		//������,����ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //����һ��ֹͣ����
    return 0;       
}

//IICдһ���ֽ� 
//devaddr:����IIC��ַ
//reg:�Ĵ�����ַ
//data:����
//����ֵ:0,����
//    ����,�������
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    if(IIC_Wait_Ack())          //�ȴ�Ӧ��
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(data);        //��������
    if(IIC_Wait_Ack())          //�ȴ�ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC��һ���ֽ� 
//reg:�Ĵ�����ַ 
//����ֵ:����������
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //����������ַ+д����
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    IIC_Send_Byte(reg);         //д�Ĵ�����ַ
    IIC_Wait_Ack();             //�ȴ�Ӧ��
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //����������ַ+������
    IIC_Wait_Ack();             //�ȴ�Ӧ��
    res=IIC_Read_Byte(0);		//������,����nACK  
    IIC_Stop();                 //����һ��ֹͣ����
    return res;  
}
