#ifndef __SINS_H
#define __SINS_H
#include "robomaster_common.h"
#define Axis_Num  3
#define Num  50
typedef struct
{
 float Position[Axis_Num];//位置估计量
 float Speed[Axis_Num];//速度估计量
 float Acceleration[Axis_Num];//加速度估计量
 float Pos_History[Axis_Num][Num];//历史惯导位置
 float Vel_History[Axis_Num][Axis_Num];//历史惯导速度
 float Acce_Bias[Axis_Num];//惯导加速度漂移量估计量
 float Last_Acceleration[Axis_Num];
 float Last_Speed[Axis_Num];
}SINS;


#define AcceMax     4096  //   4096
#define AcceGravity 9.80f
/*************WGS84地心坐标参考系数**************/
#define WGS84_RADIUS_EQUATOR        6378137.0f//半长轴，单位m
#define WGS84_INVERSE_FLATTENING    298.257223563f//扁率
#define WGS84_FLATTENING            (1/WGS84_INVERSE_FLATTENING)//扁率导数
#define WGS84_RADIUS_POLAR          (WGS84_RADIUS_EQUATOR*(1-WGS84_FLATTENING))//短轴
#define WGS84_ECCENTRICITY_SQUARED  (2*WGS84_FLATTENING-WGS84_FLATTENING*WGS84_FLATTENING)
/*********************************************
经度方向距离：LON_TO_CM*经度差，LON_TO_CM对应武汉地区所在纬度平面圆周长
纬度方向距离：LAT_TO_CM*纬度差，
*********************************************/

#define LON_COSINE_LOCAL 0.860460f//约等于当地纬度的余弦值，cos(Lat*DEG_TO_RAD)
//#define LAT_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI / (360.0f * 100000.0f))
//#define LON_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI / (360.0f * 100000.0f))*LON_COSINE_LOCAL
#define LAT_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI/360.0f)*100.0f
#define LON_TO_CM  (2.0f * WGS84_RADIUS_EQUATOR * PI*LON_COSINE_LOCAL/360.0f)*100.0f

#define LAT_TO_M  (2.0f * WGS84_RADIUS_EQUATOR * PI/360.0f)
#define LON_TO_M  (2.0f * WGS84_RADIUS_EQUATOR * PI*LON_COSINE_LOCAL/360.0f)

#ifndef M_PI_F
 #define M_PI_F 3.141592653589793f
#endif
#ifndef PI
 # define PI M_PI_F
#endif
#ifndef M_PI_2
 # define M_PI_2 1.570796326794897f
#endif
//Single precision conversions
#define DEG_TO_RAD 0.017453292519943295769236907684886f
#define RAD_TO_DEG 57.295779513082320876798154814105f

// radius of earth in meters
#define RADIUS_OF_EARTH 6378100
// scaling factor from 1e-7 degrees to meters at equater
// == 1.0e-7 * DEG_TO_RAD * RADIUS_OF_EARTH
#define LOCATION_SCALING_FACTOR 0.011131884502145034f
// inverse of LOCATION_SCALING_FACTOR
#define LOCATION_SCALING_FACTOR_INV 89.83204953368922f



typedef struct{
   // by making alt 24 bit we can make p1 in a command 16 bit,
    // allowing an accurate angle in centi-degrees. This keeps the
    // storage cost per mission item at 15 bytes, and allows mission
    // altitudes of up to +/- 83km
    int32_t alt:24; ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;    ///< param 3 - Lattitude * 10**7
    int32_t lng;    ///< param 4 - Longitude * 10**7
}Location;

extern SINS FilterBefore_NamelessQuad,FilterAfter_NamelessQuad,Filter_Feedback_NamelessQuad;

extern float Sin_Pitch,Sin_Roll,Sin_Yaw;
extern float Cos_Pitch,Cos_Roll,Cos_Yaw;
extern float Declination;//地球磁偏角
#endif
