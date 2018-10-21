#include "calibration.h"

/**
  ******************************************************************************
  * @file		 calibration.h
  * @author  izh20
  * @version 
  * @date    2018/10/20
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */
	int16_t Mag_Offset[3]={0,0,0};
float Mag_Data[3]={0};
Vector2f MagN={0,0};
float HMC5883L_Yaw=0;
	
	
/***********磁力计中心矫正，取单轴最大、最小值平均******/
uint8_t Mag_Calibration_Flag=0,Mag_Calibration_All_Finished;
uint8_t Mag_Calibration_Finished[3]={0};
uint16_t Mag_Calibration_Makesure_Cnt=0;
uint8_t  Mag_Calibration_Mode=3;
uint16_t Mag_Calibration_Cnt=0;
float Yaw_Correct=0;
/*********************************************/
const int16_t Mag_360_define[36]={
0,10,20,30,40,50,60,70,80,90,
100,110,120,130,140,150,160,170,180,190,
200,210,220,230,240,250,260,270,280,290,
300,310,320,330,340,350
};//磁力计矫正遍历角度，确保数据采集充分
uint8_t Last_Mag_360_Flag[3][36]={0};
uint8_t Mag_360_Flag[3][36]={0};
uint16_t Mag_Is_Okay_Flag[3];
Calibration Mag;
Mag_Unit DataMag;
Mag_Unit Mag_Offset_Read={
  0,0,0,
};
//void Mag_Calibration_Check(void)
//{
//   uint16_t  i=0,j=0;
//   if(Throttle_Control==1000
//      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
//        &&Roll_Control>=30
//          &&Pitch_Control>=30)
//     Mag_Calibration_Makesure_Cnt++;

//   if(Throttle_Control==1000
//      &&Yaw_Control>=Yaw_Max*Scale_Pecent_Max
//        &&Roll_Control>=30
//          &&Pitch_Control>=30
//           &&Mag_Calibration_Makesure_Cnt>200*5//持续5S
//            &&Controler_State==Lock_Controler)//必须为上锁状态才可以进入校准模式
//           //进入磁力计校准模式
//  {
//      Bling_Mode=2;
//      Mag_Calibration_Flag=1;//磁力计校准模式
//      Mag_Calibration_Mode=3;
//      Bling_Set(&Light_1,1000,500,0.2,0,GPIOC,GPIO_Pin_4,1);
//      Bling_Set(&Light_2,1000,500,0.5,0,GPIOC,GPIO_Pin_5,1);
//      Bling_Set(&Light_3,1000,500,0.7,0,GPIOC,GPIO_Pin_10,1);
//      Mag_Calibration_Makesure_Cnt=0;
//      Mag_Calibration_All_Finished=0;//全部校准完成标志位清零
//      for(i=0;i<3;i++)
//      {
//        Mag_Calibration_Finished[i]=0;//对应面标志位清零
//        for(j=0;j<36;j++) {Mag_360_Flag[i][j]=0;}
//      }
//      Page_Number=11;
//      Reset_Accel_Calibartion(1);
//      Reset_RC_Calibartion(1);
//      Forced_Lock_Makesure_Cnt=0;
//  }

//  if(Mag_Calibration_Flag==1)
//  {
//     if(Throttle_Control==1000
//        &&Yaw_Control<=-Yaw_Max*Scale_Pecent_Max
//          &&Roll_Control==0
//            &&Pitch_Control==0) //第一面矫正
//     {
//         Mag_Calibration_Cnt++;
//         if(Mag_Calibration_Cnt>=5*20)
//         {
//            Mag_Calibration_Mode=0;
//            Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
//            Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
//            Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
//            for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
//            for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
//            for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
//            LS_Init(&Mag_LS);//清空拟合中间变量
//            Unlock_Makesure_Cnt=0;
//            Lock_Makesure_Cnt=0;
//	 }

//     }
//  else if(Throttle_Control==1000
//             &&Yaw_Control>Yaw_Max*Scale_Pecent_Max
//               &&Roll_Control==0
//                 &&Pitch_Control==0) //第二面矫正
//     {
//         Mag_Calibration_Cnt++;
//         if(Mag_Calibration_Cnt>=5*20)
//         {
//             Mag_Calibration_Mode=1;
//             Mag_Is_Okay_Flag[0]=0;//单面数据采集完成标志位置0
//             Mag_Is_Okay_Flag[1]=0;//单面数据采集完成标志位置0
//             Mag_Is_Okay_Flag[2]=0;//单面数据采集完成标志位置0
//             for(i=0;i<36;i++) Mag_360_Flag[0][i]=0;//清空采集角遍历数据点
//             for(i=0;i<36;i++) Mag_360_Flag[1][i]=0;//清空采集角遍历数据点
//             for(i=0;i<36;i++) Mag_360_Flag[2][i]=0;//清空采集角遍历数据点
//             LS_Init(&Mag_LS);//清空拟合中间变量
//             Unlock_Makesure_Cnt=0;
//             Lock_Makesure_Cnt=0;
//         }
//     }
//  else
//  {
//    Mag_Calibration_Cnt/=2;
//  }
//  if(Mag_Calibration_Cnt>=200)  Mag_Calibration_Cnt=200;

//  }

//}


