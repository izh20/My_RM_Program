/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__

#include "mytype.h"
#include "robomaster_common.h"
#define MPU_DELAY(x) HAL_Delay(x)

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	float temp;

	float wx; /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;

	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;

extern mpu_data_t mpu_data;
extern imu_t      imu1;

uint8_t   mpu_device_init(void);
void init_quaternion(void);
void mpu_get_data(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
void mpu_offset_call(void);




/************user begin1*************/
//-----IMU-----//
typedef struct
{
	float accelRaw[3];	// 加速度计原始数据
	float gyroRaw[3];     // 陀螺仪原始数据
	float accelFilter[3];	// 加速度计滤波后数据
	float gyroFilter[3];	// 陀螺仪滤波后数据
}_IMU_Tag;

typedef struct
{
 float x;
 float y;
 float z;
}Vector3f;
typedef struct
{
  Vector3f a;
  Vector3f b;
  Vector3f c;
}Matrix3f;
extern float X_Origion,Y_Origion,Z_Origion;
extern _IMU_Tag imu;

#define RtA         57.324841
#define AtR    	    0.0174533
#define Acc_G 	    0.0000610351
#define Gyro_G 	    0.0610351
#define Gyro_Gr	    0.0010653
#define DEG2RAD (PI / 180.0f)
#define RAD2DEG (180.0f / PI)
#define GYRO_CALIBRATION_COFF 0.060976f  //2000
//#define GYRO_CALIBRATION_COFF 0.030488f  //1000
//#define GYRO_CALIBRATION_COFF 0.0152672f    //500
/************user end1***************/


/************user begin2*************/
float invSqrt(float x);
void GET_MPU_DATA(void);
void Acce_Correct_Filter(void);
void GET_GYRO_DATA(void);


/*********************imu****************************/


typedef struct
{
  float q[4];
  float angle[3];
}_Attitude_Tag;
extern _Attitude_Tag att;

float constrain(float value, const float min_val, const float max_val);
void Insert_Yaw(void);
void AHRSUpdate_GraDes_TimeSync(float gx, float gy, float gz, float ax, float ay, float az);
void Vector_From_EarthFrame2BodyFrame(Vector3f *ef,Vector3f *bf);
void Vector_From_BodyFrame2EarthFrame(Vector3f *bf,Vector3f *ef);
void imuComputeRotationMatrix(void);
/************user end2***************/

/********************ist8310 begin*********************************/

typedef struct {
    uint8_t Buf[6];
    int16_t Mag_Data[3];
    float Mag_Data_Correct[3];
    float thx;
    float thy;
    int16_t x;
    int16_t y;
    int16_t z;
    float Angle_Mag;
}IST8310;

extern IST8310 Mag_IST8310;

//extern float Mag_Data[3];

#define N2 10
extern float Data_X_MAG[N2];
extern float Data_Y_MAG[N2];
extern float Data_Z_MAG[N2];

float GildeAverageValueFilter_MAG(float NewValue,float *Data);

/*******************ist8310 end************************************/
#endif


