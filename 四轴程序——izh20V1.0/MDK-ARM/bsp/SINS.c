#include "SINS.h"

SINS NamelessQuad;
SINS Origion_NamelessQuad;
SINS FilterBefore_NamelessQuad;
SINS FilterAfter_NamelessQuad;
SINS Filter_Feedback_NamelessQuad;
float SpeedDealt[3]={0};
float Sin_Pitch=0,Sin_Roll=0,Sin_Yaw=0;
float Cos_Pitch=0,Cos_Roll=0,Cos_Yaw=0;
float Baro_Climbrate=0;
float Acceleration_Length=0;
Vector2f SINS_Accel_Body={0,0};
float Acce_History[3][100]={0};
float Declination=0;//µØÇò´ÅÆ«½Ç


