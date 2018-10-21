/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.c
 * @brief      mpu6500 module driver, configurate MPU6500 and Read the Accelerator
 *             and Gyrometer data using SPI interface      
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#include "bsp_imu.h"
//#include "ist8310_reg.h" 
//#include "stm32f4xx_hal.h"
#include <math.h>
//#include "mpu6500_reg.h"
//#include "spi.h"
#include "robomaster_common.h"

/********************巴特沃斯滤波器参数****************************/
Butter_Parameter Accel_Parameter={
1,  -0.7477891782585,    0.272214937925,
0.1311064399166,   0.2622128798333,   0.1311064399166
};
Butter_Parameter Gyro_Parameter={
//200hz---51hz
1,  0.03680751639284,   0.1718123812701,
0.3021549744157,   0.6043099488315,   0.3021549744157
};
Butter_Parameter Butter_1HZ_Parameter_Acce={
  //200hz---1hz
  1,   -1.955578240315,   0.9565436765112,
  0.000241359049042, 0.000482718098084, 0.000241359049042
};
/********************巴特沃斯滤波器参数****************************/


float invSqrt(float x)
{
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;

  i = 0x5f3759df - (i>>1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));

  return y;
}


/*********************************************官方历程 begin*********************************/
#define BOARD_DOWN (1)   
//#define IST8310
#define MPU_HSPI hspi5
#define MPU_NSS_LOW HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET)
#define MPU_NSS_HIGH HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_SET)

#define Kp 2.0f                                              /* 
                                                              * proportional gain governs rate of 
                                                              * convergence to accelerometer/magnetometer 
																															*/
#define Ki 0.01f                                             /* 
                                                              * integral gain governs rate of 
                                                              * convergence of gyroscope biases 
																															*/
volatile float        q0 = 1.0f;
volatile float        q1 = 0.0f;
volatile float        q2 = 0.0f;
volatile float        q3 = 0.0f;
volatile float        exInt, eyInt, ezInt;                   /* error integral */
static volatile float gx, gy, gz, ax, ay, az, mx, my, mz;  
volatile uint32_t     last_update, now_update;               /* Sampling cycle count, ubit ms */
static uint8_t        tx, rx;
static uint8_t        tx_buff[14] = { 0xff };
uint8_t               mpu_buff[14];                          /* buffer to save imu raw data */
uint8_t               ist_buff[6];                           /* buffer to save IST8310 raw data */
mpu_data_t            mpu_data;
imu_t                 imu1={0};

/**
  * @brief  fast inverse square-root, to calculate 1/Sqrt(x)
  * @param  x: the number need to be calculated
  * @retval 1/Sqrt(x)
  * @usage  call in imu_ahrs_update() function
  */
float inv_sqrt(float x) 
{
	float halfx = 0.5f * x;
	float y     = x;
	long  i     = *(long*)&y;
	
	i = 0x5f3759df - (i >> 1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	
	return y;
}

/**
  * @brief  write a byte of data to specified register
  * @param  reg:  the address of register to be written
  *         data: data to be written
  * @retval 
  * @usage  call in ist_reg_write_by_mpu(),         
  *                 ist_reg_read_by_mpu(), 
  *                 mpu_master_i2c_auto_read_config(), 
  *                 ist8310_init(), 
  *                 mpu_set_gyro_fsr(),             
  *                 mpu_set_accel_fsr(), 
  *                 mpu_device_init() function
  */
uint8_t mpu_write_byte(uint8_t const reg, uint8_t const data)
{
    MPU_NSS_LOW;
    tx = reg & 0x7F;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    tx = data;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
  * @brief  read a byte of data from specified register
  * @param  reg: the address of register to be read
  * @retval 
  * @usage  call in ist_reg_read_by_mpu(),         
  *                 mpu_device_init() function
  */
uint8_t mpu_read_byte(uint8_t const reg)
{
    MPU_NSS_LOW;
    tx = reg | 0x80;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    MPU_NSS_HIGH;
    return rx;
}

/**
  * @brief  read bytes of data from specified register
  * @param  reg: address from where data is to be written
  * @retval 
  * @usage  call in ist8310_get_data(),         
  *                 mpu_get_data(), 
  *                 mpu_offset_call() function
  */
uint8_t mpu_read_bytes(uint8_t const regAddr, uint8_t* pData, uint8_t len)
{
    MPU_NSS_LOW;
    tx         = regAddr | 0x80;
    tx_buff[0] = tx;
    HAL_SPI_TransmitReceive(&MPU_HSPI, &tx, &rx, 1, 55);
    HAL_SPI_TransmitReceive(&MPU_HSPI, tx_buff, pData, len, 55);
    MPU_NSS_HIGH;
    return 0;
}

/**
	* @brief  write IST8310 register through MPU6500's I2C master
  * @param  addr: the address to be written of IST8310's register
  *         data: data to be written
	* @retval   
  * @usage  call in ist8310_init() function
	*/
static void ist_reg_write_by_mpu(uint8_t addr, uint8_t data)
{
    /* turn off slave 1 at first */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, addr);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, data);
    MPU_DELAY(2);
    /* turn on slave 1 with one byte transmitting */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
    /* wait longer to ensure the data is transmitted from slave 1 */
    MPU_DELAY(10);
}

/**
	* @brief  write IST8310 register through MPU6500's I2C Master
	* @param  addr: the address to be read of IST8310's register
	* @retval 
  * @usage  call in ist8310_init() function
	*/
static uint8_t ist_reg_read_by_mpu(uint8_t addr)
{
    uint8_t retval;
    mpu_write_byte(MPU6500_I2C_SLV4_REG, addr);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x80);
    MPU_DELAY(10);
    retval = mpu_read_byte(MPU6500_I2C_SLV4_DI);
    /* turn off slave4 after read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);
    return retval;
}

/**
	* @brief    initialize the MPU6500 I2C Slave 0 for I2C reading.
* @param    device_address: slave device address, Address[6:0]
	* @retval   void
	* @note     
	*/
static void mpu_master_i2c_auto_read_config(uint8_t device_address, uint8_t reg_base_addr, uint8_t data_num)
{
    /* 
	   * configure the device address of the IST8310 
     * use slave1, auto transmit single measure mode 
	   */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_REG, IST8310_R_CONFA);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV1_DO, IST8310_ODR_MODE);
    MPU_DELAY(2);

    /* use slave0,auto read data */
    mpu_write_byte(MPU6500_I2C_SLV0_ADDR, 0x80 | device_address);
    MPU_DELAY(2);
    mpu_write_byte(MPU6500_I2C_SLV0_REG, reg_base_addr);
    MPU_DELAY(2);

    /* every eight mpu6500 internal samples one i2c master read */
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x03);
    MPU_DELAY(2);
    /* enable slave 0 and 1 access delay */
    mpu_write_byte(MPU6500_I2C_MST_DELAY_CTRL, 0x01 | 0x02);
    MPU_DELAY(2);
    /* enable slave 1 auto transmit */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x80 | 0x01);
		/* Wait 6ms (minimum waiting time for 16 times internal average setup) */
    MPU_DELAY(6); 
    /* enable slave 0 with data_num bytes reading */
    mpu_write_byte(MPU6500_I2C_SLV0_CTRL, 0x80 | data_num);
    MPU_DELAY(2);
}

/**
	* @brief  Initializes the IST8310 device
	* @param  
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t ist8310_init()
{
	  /* enable iic master mode */
    mpu_write_byte(MPU6500_USER_CTRL, 0x30);
    MPU_DELAY(10);
	  /* enable iic 400khz */
    mpu_write_byte(MPU6500_I2C_MST_CTRL, 0x0d); 
    MPU_DELAY(10);

    /* turn on slave 1 for ist write and slave 4 to ist read */
    mpu_write_byte(MPU6500_I2C_SLV1_ADDR, IST8310_ADDRESS);  
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_ADDR, 0x80 | IST8310_ADDRESS);
    MPU_DELAY(10);

    /* IST8310_R_CONFB 0x01 = device rst */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01);
    MPU_DELAY(10);
    if (IST8310_DEVICE_ID_A != ist_reg_read_by_mpu(IST8310_WHO_AM_I))
        return 1;

		/* soft reset */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x01); 
    MPU_DELAY(10);

		/* config as ready mode to access register */
    ist_reg_write_by_mpu(IST8310_R_CONFA, 0x00); 
    if (ist_reg_read_by_mpu(IST8310_R_CONFA) != 0x00)
        return 2;
    MPU_DELAY(10);

		/* normal state, no int */
    ist_reg_write_by_mpu(IST8310_R_CONFB, 0x00);
    if (ist_reg_read_by_mpu(IST8310_R_CONFB) != 0x00)
        return 3;
    MPU_DELAY(10);
		
    /* config low noise mode, x,y,z axis 16 time 1 avg */
    ist_reg_write_by_mpu(IST8310_AVGCNTL, 0x24); //100100
    if (ist_reg_read_by_mpu(IST8310_AVGCNTL) != 0x24)
        return 4;
    MPU_DELAY(10);

    /* Set/Reset pulse duration setup,normal mode */
    ist_reg_write_by_mpu(IST8310_PDCNTL, 0xc0);
    if (ist_reg_read_by_mpu(IST8310_PDCNTL) != 0xc0)
        return 5;
    MPU_DELAY(10);

    /* turn off slave1 & slave 4 */
    mpu_write_byte(MPU6500_I2C_SLV1_CTRL, 0x00);
    MPU_DELAY(10);
    mpu_write_byte(MPU6500_I2C_SLV4_CTRL, 0x00);
    MPU_DELAY(10);

    /* configure and turn on slave 0 */
    mpu_master_i2c_auto_read_config(IST8310_ADDRESS, IST8310_R_XL, 0x06);
    MPU_DELAY(100);
    return 0;
}

/**
	* @brief  get the data of IST8310
  * @param  buff: the buffer to save the data of IST8310
	* @retval 
  * @usage  call in mpu_get_data() function
	*/
void ist8310_get_data(uint8_t* buff)
{
    mpu_read_bytes(MPU6500_EXT_SENS_DATA_00, buff, 6); 
}


/**
	* @brief  get the data of imu
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_get_data()
{
    mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

    mpu_data.ax   = mpu_buff[0] << 8 | mpu_buff[1];
    mpu_data.ay   = mpu_buff[2] << 8 | mpu_buff[3];
    mpu_data.az   = mpu_buff[4] << 8 | mpu_buff[5];
    mpu_data.temp = mpu_buff[6] << 8 | mpu_buff[7];

    mpu_data.gx = ((mpu_buff[8]  << 8 | mpu_buff[9])  - mpu_data.gx_offset);
    mpu_data.gy = ((mpu_buff[10] << 8 | mpu_buff[11]) - mpu_data.gy_offset);
    mpu_data.gz = ((mpu_buff[12] << 8 | mpu_buff[13]) - mpu_data.gz_offset);

    ist8310_get_data(ist_buff);
    memcpy(&mpu_data.mx, ist_buff, 6);

    memcpy(&imu1.ax, &mpu_data.ax, 6 * sizeof(int16_t));
	
    imu1.temp = 21 + mpu_data.temp / 333.87f;
	  /* 2000dps -> rad/s */
	  imu1.wx   = mpu_data.gx / 16.384f / 57.3f; 
    imu1.wy   = mpu_data.gy / 16.384f / 57.3f; 
    imu1.wz   = mpu_data.gz / 16.384f / 57.3f;
}


/**
	* @brief  set imu 6500 gyroscope measure range
  * @param  fsr: range(0,±250dps;1,±500dps;2,±1000dps;3,±2000dps)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_gyro_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_GYRO_CONFIG, fsr << 3);
}


/**
	* @brief  set imu 6050/6500 accelerate measure range
  * @param  fsr: range(0,±2g;1,±4g;2,±8g;3,±16g)
	* @retval 
  * @usage  call in mpu_device_init() function
	*/
uint8_t mpu_set_accel_fsr(uint8_t fsr)
{
  return mpu_write_byte(MPU6500_ACCEL_CONFIG, fsr << 3); 
}

uint8_t id;

/**
	* @brief  initialize imu mpu6500 and magnet meter ist3810
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
uint8_t mpu_device_init(void)
{
	MPU_DELAY(100);

	id                               = mpu_read_byte(MPU6500_WHO_AM_I);
	uint8_t i                        = 0;
	uint8_t MPU6500_Init_Data[10][2] = {{ MPU6500_PWR_MGMT_1, 0x80 },     /* Reset Device */ 
																			{ MPU6500_PWR_MGMT_1, 0x03 },     /* Clock Source - Gyro-Z */ 
																			{ MPU6500_PWR_MGMT_2, 0x00 },     /* Enable Acc & Gyro */ 
																			{ MPU6500_CONFIG, 0x03 },         /* LPF 41Hz */ 
																			{ MPU6500_GYRO_CONFIG, 0x10 },    /* +-1000dps */ 
																			{ MPU6500_ACCEL_CONFIG, 0x10 },   /* +-8G */ 
																			{ MPU6500_ACCEL_CONFIG_2, 0x02 }, /* enable LowPassFilter  Set Acc LPF */ 
																			{ MPU6500_USER_CTRL, 0x20 },};    /* Enable AUX */ 
	for (i = 0; i < 10; i++)
	{
		mpu_write_byte(MPU6500_Init_Data[i][0], MPU6500_Init_Data[i][1]);
		MPU_DELAY(1);
	}

	mpu_set_gyro_fsr(2); 		
	mpu_set_accel_fsr(2);

	ist8310_init();
	mpu_offset_call();
	Set_Cutoff_Frequency(Sampling_Freq, 30,&Gyro_Parameter);//姿态角速度反馈滤波参数
  Set_Cutoff_Frequency(Sampling_Freq, 10,&Accel_Parameter);//姿态解算加计修正滤波值
  Set_Cutoff_Frequency(Sampling_Freq, 1,&Butter_1HZ_Parameter_Acce);//传感器校准加计滤波值
	return 0;
}

/**
	* @brief  get the offset data of MPU6500
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void mpu_offset_call(void)
{
	int i;
	for (i=0; i<300;i++)
	{
		mpu_read_bytes(MPU6500_ACCEL_XOUT_H, mpu_buff, 14);

		mpu_data.ax_offset += mpu_buff[0] << 8 | mpu_buff[1];
		mpu_data.ay_offset += mpu_buff[2] << 8 | mpu_buff[3];
		mpu_data.az_offset += mpu_buff[4] << 8 | mpu_buff[5];
	
		mpu_data.gx_offset += mpu_buff[8]  << 8 | mpu_buff[9];
		mpu_data.gy_offset += mpu_buff[10] << 8 | mpu_buff[11];
		mpu_data.gz_offset += mpu_buff[12] << 8 | mpu_buff[13];

		MPU_DELAY(5);
	}
	mpu_data.ax_offset=mpu_data.ax_offset / 300;
	mpu_data.ay_offset=mpu_data.ay_offset / 300;
	mpu_data.az_offset=mpu_data.az_offset / 300;
	mpu_data.gx_offset=mpu_data.gx_offset / 300;
	mpu_data.gy_offset=mpu_data.gx_offset / 300;
	mpu_data.gz_offset=mpu_data.gz_offset / 300;
}



/**
	* @brief  Initialize quaternion
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void init_quaternion(void)
{
	int16_t hx, hy;//hz;
	
	hx = imu1.mx;
	hy = imu1.my;
	//hz = imu1.mz;
	
	#ifdef BOARD_DOWN
	if (hx < 0 && hy < 0) 
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = -0.005;
			q1 = -0.199;
			q2 = 0.979;
			q3 = -0.0089;
		}
		else
		{
			q0 = -0.008;
			q1 = -0.555;
			q2 = 0.83;
			q3 = -0.002;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if (fabs(hx / hy)>=1)   
		{
			q0 = 0.005;
			q1 = -0.199;
			q2 = -0.978;
			q3 = 0.012;
		}
		else
		{
			q0 = 0.005;
			q1 = -0.553;
			q2 = -0.83;
			q3 = -0.0023;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0012;
			q1 = -0.978;
			q2 = -0.199;
			q3 = -0.005;
		}
		else
		{
			q0 = 0.0023;
			q1 = -0.83;
			q2 = -0.553;
			q3 = 0.0023;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.0025;
			q1 = 0.978;
			q2 = -0.199;
			q3 = 0.008;			
		}
		else
		{
			q0 = 0.0025;
			q1 = 0.83;
			q2 = -0.56;
			q3 = 0.0045;
		}		
	}
	#else
		if (hx < 0 && hy < 0)
	{
		if (fabs(hx / hy) >= 1)
		{
			q0 = 0.195;
			q1 = -0.015;
			q2 = 0.0043;
			q3 = 0.979;
		}
		else
		{
			q0 = 0.555;
			q1 = -0.015;
			q2 = 0.006;
			q3 = 0.829;
		}
		
	}
	else if (hx < 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.193;
			q1 = -0.009;
			q2 = -0.006;
			q3 = 0.979;
		}
		else
		{
			q0 = -0.552;
			q1 = -0.0048;
			q2 = -0.0115;
			q3 = 0.8313;
		}
		
	}
	else if (hx > 0 && hy > 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.9785;
			q1 = 0.008;
			q2 = -0.02;
			q3 = 0.195;
		}
		else
		{
			q0 = -0.9828;
			q1 = 0.002;
			q2 = -0.0167;
			q3 = 0.5557;
		}
		
	}
	else if (hx > 0 && hy < 0)
	{
		if(fabs(hx / hy) >= 1)
		{
			q0 = -0.979;
			q1 = 0.0116;
			q2 = -0.0167;
			q3 = -0.195;			
		}
		else
		{
			q0 = -0.83;
			q1 = 0.014;
			q2 = -0.012;
			q3 = -0.556;
		}		
	}
	#endif
}

/**
	* @brief  update imu AHRS
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_ahrs_update(void) 
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez, halfT;
	float tempq0,tempq1,tempq2,tempq3;

	float q0q0 = q0*q0;
	float q0q1 = q0*q1;
	float q0q2 = q0*q2;
	float q0q3 = q0*q3;
	float q1q1 = q1*q1;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q2q2 = q2*q2;   
	float q2q3 = q2*q3;
	float q3q3 = q3*q3;   

	gx = imu1.wx;
	gy = imu1.wy;
	gz = imu1.wz;
	ax = imu1.ax;
	ay = imu1.ay;
	az = imu1.az;
	mx = imu1.mx;
	my = imu1.my;
	mz = imu1.mz;

	now_update  = HAL_GetTick(); //ms
	halfT       = ((float)(now_update - last_update) / 2000.0f);
	last_update = now_update;
	
	/* Fast inverse square-root */
	norm = inv_sqrt(ax*ax + ay*ay + az*az);       
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	
	#ifdef IST8310
		norm = inv_sqrt(mx*mx + my*my + mz*mz);          
		mx = mx * norm;
		my = my * norm;
		mz = mz * norm; 
	#else
		mx = 0;
		my = 0;
		mz = 0;		
	#endif
	/* compute reference direction of flux */
	hx = 2.0f*mx*(0.5f - q2q2 - q3q3) + 2.0f*my*(q1q2 - q0q3) + 2.0f*mz*(q1q3 + q0q2);
	hy = 2.0f*mx*(q1q2 + q0q3) + 2.0f*my*(0.5f - q1q1 - q3q3) + 2.0f*mz*(q2q3 - q0q1);
	hz = 2.0f*mx*(q1q3 - q0q2) + 2.0f*my*(q2q3 + q0q1) + 2.0f*mz*(0.5f - q1q1 - q2q2);         
	bx = sqrt((hx*hx) + (hy*hy));
	bz = hz; 
	
	/* estimated direction of gravity and flux (v and w) */
	vx = 2.0f*(q1q3 - q0q2);
	vy = 2.0f*(q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	wx = 2.0f*bx*(0.5f - q2q2 - q3q3) + 2.0f*bz*(q1q3 - q0q2);
	wy = 2.0f*bx*(q1q2 - q0q3) + 2.0f*bz*(q0q1 + q2q3);
	wz = 2.0f*bx*(q0q2 + q1q3) + 2.0f*bz*(0.5f - q1q1 - q2q2);  
	
	/* 
	 * error is sum of cross product between reference direction 
	 * of fields and direction measured by sensors 
	 */
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);

	/* PI */
	if(ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;	
		ezInt = ezInt + ez * Ki * halfT;
		
		gx = gx + Kp*ex + exInt;
		gy = gy + Kp*ey + eyInt;
		gz = gz + Kp*ez + ezInt;
	}
	
	tempq0 = q0 + (-q1*gx - q2*gy - q3*gz) * halfT;
	tempq1 = q1 + (q0*gx + q2*gz - q3*gy) * halfT;
	tempq2 = q2 + (q0*gy - q1*gz + q3*gx) * halfT;
	tempq3 = q3 + (q0*gz + q1*gy - q2*gx) * halfT;  

	/* normalise quaternion */
	norm = inv_sqrt(tempq0*tempq0 + tempq1*tempq1 + tempq2*tempq2 + tempq3*tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
	
	imu_attitude_update();//update imu attitude  
}

/**
	* @brief  update imu attitude
  * @param  
	* @retval 
  * @usage  call in main() function
	*/
void imu_attitude_update(void)
{
	/* yaw    -pi----pi */
	imu1.yaw = -atan2(2*q1*q2 + 2*q0*q3, -2*q2*q2 - 2*q3*q3 + 1)* 57.3; 
	/* pitch  -pi/2----pi/2 */
	imu1.pit = -asin(-2*q1*q3 + 2*q0*q2)* 57.3;   
	/* roll   -pi----pi  */	
	imu1.rol =  atan2(2*q2*q3 + 2*q0*q1, -2*q1*q1 - 2*q2*q2 + 1)* 57.3;
}








/*******************************************无名飞控程序  mpu6500*********************************/
float  Y_g_off = -100,X_g_off = 40,Z_g_off =0;
float  X_w_off =0,Y_w_off =0,Z_w_off =0;
float K[3]={1.0,1.0,1.0};//默认标度(量程)误差
float B[3]={0,0,0};//默认零位误差
//********************************************************
float  X_g,Y_g,Z_g;
float  X_w,Y_w,Z_w;
float  X_g_av,Y_g_av,Z_g_av;//可用的加速度计值
float  X_w_av,Y_w_av,Z_w_av;//可用的陀螺仪值
_IMU_Tag imu={0};




float X_Origion,Y_Origion,Z_Origion;
void Accel_Filter(void)
{	
				
        imu.accelRaw[0] = mpu_data.ax;
        imu.accelRaw[1] = mpu_data.ay;
        imu.accelRaw[2] = mpu_data.az;
        Acce_Correct_Filter();
        X_Origion=K[0]*imu.accelRaw[0]-B[0]*One_G_TO_Accel;//经过椭球校正后的三轴加速度量
        Y_Origion=K[1]*imu.accelRaw[1]-B[1]*One_G_TO_Accel;
        Z_Origion=K[2]*imu.accelRaw[2]-B[2]*One_G_TO_Accel;
        FilterBefore_NamelessQuad.Acceleration[_YAW]=
                      -Sin_Roll* X_Origion
                        + Sin_Pitch *Cos_Roll *Y_Origion
                           + Cos_Pitch * Cos_Roll *Z_Origion;
        FilterBefore_NamelessQuad.Acceleration[_PITCH]=
                   Cos_Yaw* Cos_Roll * X_Origion
                        +(Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw) * Y_Origion
                          +(Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw) * Z_Origion;
        FilterBefore_NamelessQuad.Acceleration[_ROLL]=
                   Sin_Yaw* Cos_Roll * X_Origion
                        +(Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw) * Y_Origion
                          + (Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw) * Z_Origion;
        FilterBefore_NamelessQuad.Acceleration[_YAW]*=AcceGravity/AcceMax;
        FilterBefore_NamelessQuad.Acceleration[_YAW]-=AcceGravity;
        FilterBefore_NamelessQuad.Acceleration[_YAW]*=100;//加速度cm/s^2
        FilterBefore_NamelessQuad.Acceleration[_PITCH]*=AcceGravity/AcceMax;
        FilterBefore_NamelessQuad.Acceleration[_PITCH]*=100;//加速度cm/s^2
        FilterBefore_NamelessQuad.Acceleration[_ROLL]*=AcceGravity/AcceMax;
        FilterBefore_NamelessQuad.Acceleration[_ROLL]*=100;//加速度cm/s^2
        Acce_Control_Filter();//加速度滤波，用于惯导、加速度控制反馈量
	/* 加速度计Butterworth滤波 */
        X_g_av=MPU_LPF(X_Origion,&Accel_BufferData[0],&Accel_Parameter);
        Y_g_av=MPU_LPF(Y_Origion,&Accel_BufferData[1],&Accel_Parameter);
        Z_g_av=MPU_LPF(Z_Origion,&Accel_BufferData[2],&Accel_Parameter);
}

float MagTemp[3]={0};
IST8310 Mag_IST8310;
Vector3f DCM_Gyro,DCM_Acc;
float Gyro_Range_Scale=0,Gyro_Range_Offset_Gain=2000;
void GET_MPU_DATA(void)
{
  //{Gyro_Range_Scale=0.007633f;Gyro_Range_Offset_Gain=250;}
  //{Gyro_Range_Scale=0.015267f;Gyro_Range_Offset_Gain=500;}
  {Gyro_Range_Scale=0.030487f;Gyro_Range_Offset_Gain=1000;}
  //{Gyro_Range_Scale=0.060975f;Gyro_Range_Offset_Gain=2000;}
	mpu_get_data();
  Accel_Filter();
  GET_GYRO_DATA();
	/**********************ist8310 begin**************/
	
	
	ist8310_get_data(ist_buff);
	 memcpy(&Mag_IST8310.Mag_Data[0], ist_buff, 6);
	#ifdef MAG_REVERSE_SIDE//重新映射磁力计三轴数据
   Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
   Mag_IST8310.y = -Mag_IST8310.Mag_Data[1];
   Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
#else
   Mag_IST8310.x = Mag_IST8310.Mag_Data[0];
   Mag_IST8310.y = Mag_IST8310.Mag_Data[1];
   Mag_IST8310.z = Mag_IST8310.Mag_Data[2];
#endif
   DataMag.x=Mag_IST8310.x;
   DataMag.y=Mag_IST8310.y;
   DataMag.z=Mag_IST8310.z;
   MagTemp[0]=GildeAverageValueFilter_MAG(Mag_IST8310.x-Mag_Offset[0],Data_X_MAG);//滑动窗口滤波
   MagTemp[1]=GildeAverageValueFilter_MAG(Mag_IST8310.y-Mag_Offset[1],Data_Y_MAG);
   MagTemp[2]=GildeAverageValueFilter_MAG(Mag_IST8310.z-Mag_Offset[2],Data_Z_MAG);
	 
	  Mag_Data[0]=Mag_IST8310.Mag_Data_Correct[0]=MagTemp[0];
   Mag_Data[1]=Mag_IST8310.Mag_Data_Correct[1]=MagTemp[1];
   Mag_Data[2]=Mag_IST8310.Mag_Data_Correct[2]=MagTemp[2];
   
   /************磁力计倾角补偿*****************/
   MagN.x=Mag_IST8310.thx = MagTemp[0] * Cos_Roll+ MagTemp[2] * Sin_Roll;
   MagN.y=Mag_IST8310.thy = MagTemp[0] * Sin_Pitch*Sin_Roll
                    +MagTemp[1] * Cos_Pitch
                    -MagTemp[2] * Cos_Roll*Sin_Pitch;
   /***********反正切得到磁力计观测角度*********/
   Mag_IST8310.Angle_Mag=atan2(Mag_IST8310.thx,Mag_IST8310.thy)*57.296;
	 
	 
	/**********************ist8310 end****************/
  DCM_Acc.x=X_Origion;
  DCM_Acc.y=Y_Origion;
  DCM_Acc.z=Z_Origion;
  DCM_Gyro.x=X_w_av*GYRO_CALIBRATION_COFF*DEG2RAD;
  DCM_Gyro.y=Y_w_av*GYRO_CALIBRATION_COFF*DEG2RAD;
  DCM_Gyro.z=Z_w_av*GYRO_CALIBRATION_COFF*DEG2RAD;
}


int16_t Acce_Correct[3]={0};//用于矫正加速度量，截至频率很低
uint8_t Acce_Correct_Update_Flag=0;
Butter_BufferData Butter_Buffer_Correct[3];
void Acce_Correct_Filter(void)
{
   Acce_Correct[0]=Int_Sort(LPButterworth(imu.accelRaw[0],
                    &Butter_Buffer_Correct[0],&Butter_1HZ_Parameter_Acce));
   Acce_Correct[1]=Int_Sort(LPButterworth(imu.accelRaw[1]
                    ,&Butter_Buffer_Correct[1],&Butter_1HZ_Parameter_Acce));
   Acce_Correct[2]=Int_Sort(LPButterworth(imu.accelRaw[2]
                    ,&Butter_Buffer_Correct[2],&Butter_1HZ_Parameter_Acce));
   Acce_Correct_Update_Flag=1;
}


void GET_GYRO_DATA(void)//角速度低通滤波后用于姿态解算
{
	X_w  = mpu_data.gx;
	Y_w  = mpu_data.gy;
	Z_w  = mpu_data.gz;
        X_w_av=MPU_LPF(X_w,
                        &Gyro_BufferData[0],
                        &Gyro_Parameter
                        );
        Y_w_av=MPU_LPF(Y_w,
                        &Gyro_BufferData[1],
                        &Gyro_Parameter
                        );
        Z_w_av=MPU_LPF(Z_w,
                        &Gyro_BufferData[2],
                        &Gyro_Parameter
                        );
}

/***************************mpu6500 end******************************/

/***************************ist8310 begin***************************/
//ist8310_get_data(ist_buff);
//    memcpy(&mpu_data.mx, ist_buff, 6);
float Data_X_MAG[N2];
float Data_Y_MAG[N2];
float Data_Z_MAG[N2];
float GildeAverageValueFilter_MAG(float NewValue,float *Data)
{
	float max,min;
	float sum;
	unsigned char i;
	Data[0]=NewValue;
	max=Data[0];
	min=Data[0];
	sum=Data[0];
	for(i=N2-1;i!=0;i--)
	{
	  if(Data[i]>max) max=Data[i];
	  else if(Data[i]<min) min=Data[i];
	  sum+=Data[i];
	  Data[i]=Data[i-1];
	}
	 i=N2-2;
	 sum=sum-max-min;
	 sum=sum/i;
	 return(sum);
}


/***************************ist8310 end******************************/



/********************************imu begin********************************/

Vector3f_Body Circle_Angle;
float Yaw=0,Pitch=0,Roll=0;//四元数计算出的角度
float Yaw_Gyro=0,Pitch_Gyro=0,Roll_Gyro=0;
float Yaw_Gyro_Earth_Frame=0;
float Mag_Yaw=0;
void Insert_Yaw(void)
{
#ifdef IMU_BOARD_NC686
  if(Extern_Mag_Work_Flag==1) Mag_Yaw=HMC5883L_Yaw;
  else Mag_Yaw=Mag_IST8310.Angle_Mag;
#endif
Mag_Yaw=Mag_IST8310.Angle_Mag;//只有ist8310
#ifdef IMU_BOARD_GY86
    Mag_Yaw=HMC5883L_Yaw;
#endif
}

_Attitude_Tag att =
{
	{1.0f, 0.0f, 0.0f, 0.0f},
	{0.0f, 0.0f, 0.0f}
};

float constrain(float value, const float min_val, const float max_val)
{
  if(value>=max_val)  value=max_val;
  if(value<=min_val)  value=min_val;
  return value;
}


float q0_DCM  = 1.0f, q1_DCM  = 0.0f, q2_DCM  = 0.0f, q3_DCM  = 0.0f;
float rMat[3][3];
float sqf(float x) {return ((x)*(x));}
void imuComputeRotationMatrix(void)
{
/*    float q1q1,q2q2,q3q3;
    float q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;

    q0_DCM=att.q[0];
    q1_DCM=att.q[1];
    q2_DCM=att.q[2];
    q3_DCM=att.q[3];

    q1q1 = sqf(q1_DCM );
    q2q2 = sqf(q2_DCM );
    q3q3 = sqf(q3_DCM );

    q0q1 = q0_DCM  * q1_DCM ;
    q0q2 = q0_DCM  * q2_DCM ;
    q0q3 = q0_DCM  * q3_DCM ;
    q1q2 = q1_DCM  * q2_DCM ;
    q1q3 = q1_DCM  * q3_DCM ;
    q2q3 = q2_DCM  * q3_DCM ;

    rMat[0][0] = 1.0f - 2.0f * q2q2 - 2.0f * q3q3;
    rMat[0][1] = 2.0f * (q1q2 -q0q3);
    rMat[0][2] = 2.0f * (q1q3 +q0q2);

    rMat[1][0] = 2.0f * (q1q2 +q0q3);
    rMat[1][1] = 1.0f - 2.0f * q1q1 - 2.0f * q3q3;
    rMat[1][2] = 2.0f * (q2q3 -q0q1);

    rMat[2][0] = 2.0f * (q1q3 -q0q2);
    rMat[2][1] = 2.0f * (q2q3 +q0q1);
    rMat[2][2] = 1.0f - 2.0f * q1q1 - 2.0f * q2q2;
  */
   Sin_Pitch=sin(Pitch* DEG2RAD);
   Cos_Pitch=cos(Pitch* DEG2RAD);
   Sin_Roll=sin(Roll* DEG2RAD);
   Cos_Roll=cos(Roll* DEG2RAD);
   Sin_Yaw=sin(Yaw* DEG2RAD);
   Cos_Yaw=cos(Yaw* DEG2RAD);

   rMat[0][0]=Cos_Yaw* Cos_Roll;
   rMat[0][1]=Sin_Pitch*Sin_Roll*Cos_Yaw-Cos_Pitch * Sin_Yaw;
   rMat[0][2]=Sin_Pitch * Sin_Yaw+Cos_Pitch * Sin_Roll * Cos_Yaw;

   rMat[1][0]=Sin_Yaw * Cos_Roll;
   rMat[1][1]=Sin_Pitch * Sin_Roll * Sin_Yaw +Cos_Pitch * Cos_Yaw;
   rMat[1][2]=Cos_Pitch * Sin_Roll * Sin_Yaw - Sin_Pitch * Cos_Yaw;

   rMat[2][0]=-Sin_Roll;
   rMat[2][1]= Sin_Pitch * Cos_Roll;
   rMat[2][2]= Cos_Pitch * Cos_Roll;
}



void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef)
{
      ef->x=rMat[0][0]*bf->x+rMat[0][1]*bf->y+rMat[0][2]*bf->z;
      ef->y=rMat[1][0]*bf->x+rMat[1][1]*bf->y+rMat[1][2]*bf->z;
      ef->z=rMat[2][0]*bf->x+rMat[2][1]*bf->y+rMat[2][2]*bf->z;
}

void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf)
{
      bf->x=rMat[0][0]*ef->x+rMat[1][0]*ef->y+rMat[2][0]*ef->z;
      bf->y=rMat[0][1]*ef->x+rMat[1][1]*ef->y+rMat[2][1]*ef->z;
      bf->z=rMat[0][2]*ef->x+rMat[1][2]*ef->y+rMat[2][2]*ef->z;
}

/******************************************************************************************
函数名:	void AHRSUpdate_GraDes(float gx, float gy, float gz, float ax, float ay, float az)
说明:	陀螺仪+加速度计梯度下降姿态融合算法
入口:	float gx	陀螺仪x分量
		float gy	陀螺仪y分量
		float gz	陀螺仪z分量
		float ax	加速度计x分量
		float ay	加速度计y分量
		float az	加速度计z分量
出口:	无
备注:	http://blog.csdn.net/nemol1990
******************************************************************************************/
float Beta_Adjust[5]={0.015,0.005,0.010,0.02,0.01};//{0.04,0.03,0.025,0.02,0.01};{0.05,0.03,0.025,0.02,0.01};
float BETADEF=0.02;
float Gyro_Length=0;//陀螺仪模长
#define Quad_Num  20
float Quad_Buf[Quad_Num][4]={0};
uint16 Quad_Delay=5;//3  10  5
float Gyro_History[3]={0};//角速度
float Gyro_Delta[3]={0};//角速度增量
float Gyro_Delta_Length=0;//角加速度模长
float Acce_Length=0;//角加速度模长
Testime IMU_Delta;
float IMU_Dt=0;
float gyro[3]={0};
float Gyro_Record[3][10]={0};
#define TimeSync_Cnt  9  
void AHRSUpdate_GraDes_TimeSync(float gx, float gy, float gz, float ax, float ay, float az)
{
	float recipNorm;					// 平方根
	float s0, s1, s2, s3;					// 梯度下降算子求出来的姿态
	float qDot1, qDot2, qDot3, qDot4;			// 四元数微分方程求得的姿态
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;
	float delta;
        uint16 i=0;
        //float Tmep_Acce_Length=0;
        static Vector3f accel[Quad_Num];
        static Vector4q quad_buffer[Quad_Num];
        static uint16_t sync_cnt=0;
        Test_Period(&IMU_Delta);
        IMU_Dt=IMU_Delta.Time_Delta/1000.0;
        Insert_Yaw();//根具不同传感器模块，选择对应磁力计
        sync_cnt++;
        if(sync_cnt>=4)//4*5=20ms滑动一次
        {
          for(i=Quad_Num-1;i>0;i--)//将四元数历史值保存起来,20*20=400ms
          {
            quad_buffer[i].q0=quad_buffer[i-1].q0;
            quad_buffer[i].q1=quad_buffer[i-1].q1;
            quad_buffer[i].q2=quad_buffer[i-1].q2;
            quad_buffer[i].q3=quad_buffer[i-1].q3;
            accel[i].x=accel[i-1].x;
            accel[i].y=accel[i-1].y;
            accel[i].z=accel[i-1].z;
          }
            quad_buffer[0].q0=att.q[0];
            quad_buffer[0].q1=att.q[1];
            quad_buffer[0].q2=att.q[2];
            quad_buffer[0].q3=att.q[3];
            sync_cnt=0;
        }
        
        
          accel[0].x=ax;
          accel[0].y=ay;
          accel[0].z=az;

           for(i=9;i>0;i--)//将四元数历史值保存起来,20*20=400ms
          {
              Gyro_Record[0][i]=Gyro_Record[0][i-1];
              Gyro_Record[1][i]=Gyro_Record[1][i-1];
              Gyro_Record[2][i]=Gyro_Record[2][i-1];
          }
              Gyro_Record[0][0]=Pitch_Gyro;
              Gyro_Record[1][0]=Roll_Gyro;
              Gyro_Record[2][0]=Yaw_Gyro;

          /**************角速度数字量转化成角度制，单位:度/秒(deg/s)*************/
          gx*=Gyro_Range_Scale;
          gy*=Gyro_Range_Scale;
          gz*=Gyro_Range_Scale;
          /************角速度赋值，用于姿态控制内环,角速度反馈*************/

          Pitch_Gyro=X_w_av*Gyro_Range_Scale;
          Roll_Gyro=Y_w_av*Gyro_Range_Scale;
          Yaw_Gyro=Z_w_av*Gyro_Range_Scale;
          //{-sinθ          cosθsin Φ                          cosθcosΦ                   }
          Yaw_Gyro_Earth_Frame=-Sin_Roll*gx+ Cos_Roll*Sin_Pitch *gy+ Cos_Pitch * Cos_Roll *gz;
          Gyro_Delta[0]=(Gyro_Record[0][0]-Gyro_Record[0][1]);
          Gyro_Delta[1]=(Gyro_Record[1][0]-Gyro_Record[1][1]);
          Gyro_Delta[2]=(Gyro_Record[2][0]-Gyro_Record[2][1]);
          //角加速度模长
          Gyro_Delta_Length=sqrt(Gyro_Delta[0]*Gyro_Delta[0]
                                 +Gyro_Delta[1]*Gyro_Delta[1]
                                         +Gyro_Delta[2]*Gyro_Delta[2]);
          //角速度模长
          Gyro_Length=sqrt(Yaw_Gyro*Yaw_Gyro
                                 +Pitch_Gyro*Pitch_Gyro
                                         +Roll_Gyro*Roll_Gyro);//单位deg/s
          /* 转换为弧度制，用于姿态更新*/
          gx = gx * PI / 180;
          gy = gy * PI / 180;
          gz = gz * PI / 180;
          /* 四元数微分方程计算本次待矫正四元数 */
          qDot1 = 0.5f * (-att.q[1] * gx- att.q[2] * gy - att.q[3] * gz);
          qDot2 = 0.5f * (att.q[0] * gx + att.q[2] * gz - att.q[3] * gy);
          qDot3 = 0.5f * (att.q[0] * gy - att.q[1] * gz + att.q[3] * gx);
          qDot4 = 0.5f * (att.q[0] * gz + att.q[1] * gy - att.q[2] * gx);
	/* 加速度计输出有效时,利用加速度计补偿陀螺仪 */
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
	{
//                if(Effective_Gravity_Acceleration(GPS_Sate_Num,GPS_Quality))//GPS提取运动加速度
//                {
//                  ax=accel[9].x-Body_Motion_Acceleration.x;//剔除运动加速度
//                  ay=accel[9].y-Body_Motion_Acceleration.y;
//                  az=accel[9].z-Body_Motion_Acceleration.z;
//                }
//                else
//                {
                  ax=accel[9].x;
                  ay=accel[9].y;
                  az=accel[9].z;
               // }
                recipNorm=invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;
		/* 避免重复运算 */
		_2q0 = 2.0f * quad_buffer[TimeSync_Cnt].q0;
		_2q1 = 2.0f * quad_buffer[TimeSync_Cnt].q1;
		_2q2 = 2.0f * quad_buffer[TimeSync_Cnt].q2;
		_2q3 = 2.0f * quad_buffer[TimeSync_Cnt].q3;
		_4q0 = quad_buffer[TimeSync_Cnt].q0;
		_4q1 = 4.0f * quad_buffer[TimeSync_Cnt].q1;
		_4q2 = 4.0f * quad_buffer[TimeSync_Cnt].q2;
		_8q1 = 8.0f * quad_buffer[TimeSync_Cnt].q1;
		_8q2 = 8.0f * quad_buffer[TimeSync_Cnt].q2;
		q0q0 = quad_buffer[9].q0 * quad_buffer[TimeSync_Cnt].q0;
		q1q1 = quad_buffer[9].q1 * quad_buffer[TimeSync_Cnt].q1;
		q2q2 = quad_buffer[9].q2 * quad_buffer[TimeSync_Cnt].q2;
		q3q3 = quad_buffer[9].q3 * quad_buffer[TimeSync_Cnt].q3;

		/* 梯度下降算法,计算误差函数的梯度 */
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * quad_buffer[TimeSync_Cnt].q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * quad_buffer[TimeSync_Cnt].q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * quad_buffer[TimeSync_Cnt].q3 - _2q1 * ax + 4.0f * q2q2 * quad_buffer[TimeSync_Cnt].q2 - _2q2 * ay;

		/* 梯度归一化 */
		recipNorm=invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

                BETADEF=Beta_Adjust[1]+0.01*Gyro_Length*IMU_Dt;
                BETADEF=constrain(BETADEF,0,0.04);
                //Tmep_Acce_Length=constrain(Acceleration_Length,0,1000);//正常悬停在500以内
                //BETADEF=Beta_Adjust[0]-0.01*Tmep_Acce_Length/1000;//动态步长
                qDot1 -= BETADEF * s0;
                qDot2 -= BETADEF * s1;
                qDot3 -= BETADEF * s2;
                qDot4 -= BETADEF * s3;
	}
		/* 补偿由四元数微分方程引入的姿态误差 */
		/* 将四元数姿态导数积分,得到当前四元数姿态 */
		/* 二阶毕卡求解微分方程 */
		delta = (IMU_Dt * gx) * (IMU_Dt * gx) + (IMU_Dt * gy) * (IMU_Dt * gy) + (IMU_Dt * gz) * (IMU_Dt * gz);
		att.q[0] = (1.0f - delta / 8.0f) * att.q[0] + qDot1 * IMU_Dt;
		att.q[1] = (1.0f - delta / 8.0f) * att.q[1] + qDot2 * IMU_Dt;
		att.q[2] = (1.0f - delta / 8.0f) * att.q[2] + qDot3 * IMU_Dt;
		att.q[3] = (1.0f - delta / 8.0f) * att.q[3] + qDot4 * IMU_Dt;
		/* 单位化四元数 */
		recipNorm=invSqrt(att.q[0] * att.q[0] + att.q[1] * att.q[1] + att.q[2] * att.q[2] + att.q[3] * att.q[3]);
		att.q[0] *= recipNorm;
		att.q[1] *= recipNorm;
		att.q[2] *= recipNorm;
		att.q[3] *= recipNorm;
		/* 四元数到欧拉角转换,转换顺序为Z-Y-X,参见<Representing Attitude: Euler Angles, Unit Quaternions, and Rotation Vectors>.pdf一文,P24 */
		Pitch= atan2(2.0f * att.q[2] * att.q[3] + 2.0f * att.q[0] * att.q[1], -2.0f * att.q[1] * att.q[1] - 2.0f * att.q[2]* att.q[2] + 1.0f) * RAD2DEG;// Pitch
		Roll= asin(2.0f * att.q[0]* att.q[2]-2.0f * att.q[1] * att.q[3]) * RAD2DEG;									// Roll
		//att.angle[YAW] = atan2(2.0f * att.q[1] * att.q[2] + 2.0f * att.q[0] * att.q[3], -2.0f * att.q[3] * att.q[3] - 2.0f * att.q[2] * att.q[2] + 1.0f) * RAD2DEG;// Yaw
						/*偏航角一阶互补*/
		//att.angle[_YAW]+=Yaw_Gyro*dt;
                att.angle[_YAW]+=Yaw_Gyro_Earth_Frame*IMU_Dt;
		if((Mag_Yaw>90 && att.angle[_YAW]<-90)
		   || (Mag_Yaw<-90 && att.angle[_YAW]>90))
                att.angle[_YAW] = -att.angle[_YAW] * 0.98f + Mag_Yaw * 0.02f;
                else att.angle[_YAW] = att.angle[_YAW] * 0.98f + Mag_Yaw * 0.02f;

		if(att.angle[_YAW]<0)   Yaw=att.angle[_YAW]+360;
		else Yaw=att.angle[_YAW];
                 
               // if(GPS_Home_Set==1)  Yaw=Yaw-Declination;//如果GPS home点已设置，获取当地磁偏角，得到地理真北
	        imuComputeRotationMatrix();

                 Circle_Angle.Pit+=Pitch_Gyro*IMU_Dt;
                 Circle_Angle.Rol+=Roll_Gyro*IMU_Dt;
                 Circle_Angle.Yaw+=Yaw_Gyro*IMU_Dt;
                 if(Circle_Angle.Pit<0)   Circle_Angle.Pit+=360;
                 if(Circle_Angle.Pit>360) Circle_Angle.Pit-=360;
                 if(Circle_Angle.Rol<0)   Circle_Angle.Rol+=360;
                 if(Circle_Angle.Rol>360) Circle_Angle.Rol-=360;
                 if(Circle_Angle.Yaw<0)   Circle_Angle.Yaw+=360;
                 if(Circle_Angle.Yaw>360) Circle_Angle.Yaw-=360;
}



/*************************imu end*****************************************/
