/**
  ******************************************************************************
  * @file		 robomaster_filter.h
  * @author  ÖÜºã
  * @version V1.0.0
  * @date    2018/10/13
  * @brief   imuÂË²¨Æ÷
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/


#ifndef __ROBOMASTER_FILTER
#define __ROBOMASTER_FILTER
#include "robomaster_common.h"
#define Sampling_Freq 200//200hz
#define PI 3.1415926535898
#define M_PI_F 3.141592653589793f
typedef struct
{
 float Input_Butter[3];
 float Output_Butter[3];
}Butter_BufferData;

typedef struct
{
  float a[3];
  float b[3];
}Butter_Parameter;

typedef struct
{
 float x;
 float y;
 float z;
}Vector3f;

extern Butter_BufferData Gyro_BufferData[3];
extern Butter_BufferData Butter_5HZ_Buffer_Acce[3];
extern float Acce_Control[3],Acce_Control_Feedback[3],Acce_SINS[3];


extern Butter_Parameter Butter_80HZ_Parameter_Acce,Butter_60HZ_Parameter_Acce,Butter_51HZ_Parameter_Acce,
                 Butter_30HZ_Parameter_Acce,Butter_20HZ_Parameter_Acce,Butter_15HZ_Parameter_Acce,
                 Butter_10HZ_Parameter_Acce,Butter_5HZ_Parameter_Acce,Butter_2HZ_Parameter_Acce;


float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
void Acce_Control_Filter(void);
float LPButterworth(float curr_input,Butter_BufferData *Buffer,Butter_Parameter *Parameter);

float MPU_LPF(float curr_inputer,
               Butter_BufferData *Buffer,
               Butter_Parameter *Parameter);
float set_lpf_alpha(int16_t cutoff_frequency, float time_step);
void Set_Cutoff_Frequency(float sample_frequent, float cutoff_frequent,Butter_Parameter *LPF);
void Butterworth_Parameter_Init(void);
#endif
