#ifndef __CALIBRATION_H
#define __CALIBRATION_H
#include "robomaster_common.h"



#define AcceMax_1G      4096
#define GRAVITY_MSS     9.80665f
#define ACCEL_TO_1G     GRAVITY_MSS/AcceMax_1G
#define One_G_TO_Accel  AcceMax_1G/GRAVITY_MSS


typedef struct
{
 float x;
 float y;
 float z;
}Acce_Unit;

typedef struct
{
 float x;
 float y;
 float z;
}Mag_Unit;


typedef struct {
    int16_t x_max;
    int16_t y_max;
    int16_t z_max;
    int16_t x_min;
    int16_t y_min;
    int16_t z_min;
    float x_offset;
    float y_offset;
    float z_offset;
}Calibration;

typedef struct
{
 float x;
 float y;
}Vector2f;

//extern Acce_Unit acce_sample[6];
//extern uint8_t  Mag_Calibration_Mode;
//extern uint8_t flight_direction;
//extern Acce_Unit Accel_Offset_Read,Accel_Scale_Read;
extern Mag_Unit DataMag;
//extern Mag_Unit Mag_Offset_Read;
//extern Calibration Mag;
//extern uint8_t Mag_360_Flag[3][36];
//extern uint16_t Mag_Is_Okay_Flag[3];
//extern float Yaw_Correct;
extern Vector2f MagN;
extern int16_t Mag_Offset[3];
extern float Mag_Data[3];
//extern float HMC5883L_Yaw;
////extern Vector_RC  RC_Calibration[8];

//extern float mag_a,mag_b,mag_c,mag_r;
//extern uint32_t ESC_Calibration_Flag;

#endif

