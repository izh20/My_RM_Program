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
    IIC_Init();     //初始化IIC总线
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    delay_ms(100);  //延时100ms
	HAL_GPIO_WritePin(LED1_GPIO_Port, GPIO_PIN_14, GPIO_PIN_RESET);
		
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
    MPU_Set_Rate(200);						       	 	//设置采样率200Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计
    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    if(res==MPU6500_ID1||res==MPU6500_ID2) //器件ID正确
    {
			
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
        MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
		MPU_Set_Rate(200);						       	//设置采样率为200Hz   
    }else return 1;
 
    res=MPU_Read_Byte(AK8963_ADDR,MAG_WIA);    			//读取AK8963 ID   
    if(res==AK8963_ID)
    {
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL2,0X01);		//复位AK8963
		delay_ms(50);
			
        MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11);		//设置AK8963为单次测量
			
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





//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Gyro_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_GYRO_CFG_REG,(fsr<<3)|3);//设置陀螺仪满量程范围  
}
//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Accel_Fsr(u8 fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_LPF(u16 lpf)
{
	u8 data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
u8 MPU_Set_Rate(u16 rate)
{
	u8 data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}

/*/////////////////////////////////////////////
*@功能：补偿陀螺仪漂移
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



//得到温度值
//返回值:温度值(扩大了100倍)
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
//得到陀螺仪值(原始值)，对源数据平均值滤波，并调整陀螺仪方位
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
*@功能：获得陀螺仪数据，单位弧度每秒
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

//得到加速度值(原始值)，低通滤波并调整加速度计方位
//gx,gy,gz:陀螺仪x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
*@功能：获得加速度计数据，单位g，并对加速度计进行补偿
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
//得到磁力计值(原始值)，平均滤波并调整方位
//mx,my,mz:磁力计x,y,z轴的原始读数(带符号)
//返回值:0,成功
//    其他,错误代码
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
	MPU_Write_Byte(AK8963_ADDR,MAG_CNTL1,0X11); //AK8963每次读完以后都需要重新设置为单次测量模式
    return res;
}

/*//////////////////////////////////////////////////
*@功能：获得磁力计数据，单位高斯，并对磁力计进行补偿
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

//IIC连续写
//addr:器件地址 
//reg:寄存器地址
//len:写入长度
//buf:数据区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{
    u8 i;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    for(i=0;i<len;i++)
    {
        IIC_Send_Byte(buf[i]);  //发送数据
        if(IIC_Wait_Ack())      //等待ACK
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_Stop();
    return 0;
} 

//IIC连续读
//addr:器件地址
//reg:要读取的寄存器地址
//len:要读取的长度
//buf:读取到的数据存储区
//返回值:0,正常
//    其他,错误代码
u8 MPU_Read_Len(u8 addr,u8 reg,u8 len,u8 *buf)
{ 
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    while(len)
    {
        if(len==1)*buf=IIC_Read_Byte(0);//读数据,发送nACK 
		else *buf=IIC_Read_Byte(1);		//读数据,发送ACK  
		len--;
		buf++;  
    }
    IIC_Stop();                 //产生一个停止条件
    return 0;       
}

//IIC写一个字节 
//devaddr:器件IIC地址
//reg:寄存器地址
//data:数据
//返回值:0,正常
//    其他,错误代码
u8 MPU_Write_Byte(u8 addr,u8 reg,u8 data)
{
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    if(IIC_Wait_Ack())          //等待应答
    {
        IIC_Stop();
        return 1;
    }
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(data);        //发送数据
    if(IIC_Wait_Ack())          //等待ACK
    {
        IIC_Stop();
        return 1;
    }
    IIC_Stop();
    return 0;
}

//IIC读一个字节 
//reg:寄存器地址 
//返回值:读到的数据
u8 MPU_Read_Byte(u8 addr,u8 reg)
{
    u8 res;
    IIC_Start();
    IIC_Send_Byte((addr<<1)|0); //发送器件地址+写命令
    IIC_Wait_Ack();             //等待应答
    IIC_Send_Byte(reg);         //写寄存器地址
    IIC_Wait_Ack();             //等待应答
	IIC_Start();                
    IIC_Send_Byte((addr<<1)|1); //发送器件地址+读命令
    IIC_Wait_Ack();             //等待应答
    res=IIC_Read_Byte(0);		//读数据,发送nACK  
    IIC_Stop();                 //产生一个停止条件
    return res;  
}
