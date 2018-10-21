#ifndef __ROBOMASTER_COMMON
#define __ROBOMASTER_COMMON

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mytype.h"
#include <math.h>
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "Remote_Control.h"
#include "bsp_imu.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "robomaster_vcan.h"
#include "robomaster_control.h"
#include "robomaster_task.h"
#include "pwm.h"
#include "filter.h"
#include "SINS.h"
#include "calibration.h"
/* USER CODE END Includes */
#ifndef PI
#define PI 3.1415926535898
#endif
#define CNTLCYCLE  0.005f
#define AHRS_DT  0.005f
#define Delta 0.005f
#define Dt 0.005f
#define _YAW    0
#define _PITCH  1
#define _ROLL   2

#define Sampling_Freq 200//200hz
//³¬Éù²¨¶¨¸ß
#define HC_SR04  1
#define Sampling_Freq 200//200hz

#define Int_Sort    (int16_t)
#define ABS(X)  (((X)>0)?(X):-(X))
#define MAX(a,b)  ((a)>(b)?(a):(b))
#define MIN(a,b)  ((a)<(b)?(a):(b))
typedef   signed           char int8;
typedef unsigned           char uint8;
typedef unsigned           char byte;
typedef   signed short     int int16;
typedef unsigned short     int uint16;

typedef struct
{
 int32_t x;
 int32_t y;
}Vector2i;

//typedef struct
//{
// float x;
// float y;
//}Vector2f;


//typedef struct
//{
// float x;
// float y;
// float z;
//}Vector3f;


typedef struct
{
 float q0;
 float q1;
 float q2;
 float q3;
}Vector4q;


typedef struct
{
 int16_t x;
 int16_t y;
 int16_t z;
}Vector3i;



typedef struct
{
 float E;
 float N;
 float U;
}Vector3_Nav;

typedef struct
{
 float E;
 float N;
}Vector2f_Nav;



typedef struct
{
 int32_t lat;
 int32_t lng;
}Vector2_Nav;



typedef struct
{
 float x;
 float y;
 float z;
}Vector3_Body;


typedef struct
{
 float Pit;
 float Rol;
}Vector2_Ang;


typedef struct
{
 float Pit;
 float Rol;
}Vector2_Body;


typedef struct
{
 float Pit;
 float Rol;
 float Yaw;
}Vector3f_Body;


typedef struct
{
 float North;
 float East;
}Vector2_Earth;

//typedef struct
//{
//  Vector3f a;
//  Vector3f b;
//  Vector3f c;
//}Matrix3f;







#endif

