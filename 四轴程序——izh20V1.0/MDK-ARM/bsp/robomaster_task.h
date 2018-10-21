#ifndef __ROBOMASTER_TASK
#define __ROBOMASTER_TASK
#include "robomaster_common.h"


typedef struct
{
  float Last_Time;
  float Now_Time;
  float Time_Delta;
  uint16_t Time_Delta_INT;//µ¥Î»ms
}Testime;


void task(void);


void Test_Period( Testime *Time_Lab);
#endif

