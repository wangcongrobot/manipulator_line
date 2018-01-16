#ifndef PARAMETERS_H_INCLUDED
#define PARAMETERS_H_INCLUDED

#include "utilities.h"

const double Pi = 3.14159265358979;
const double EPSINON = 1e-5;

//signed short limitMin[7]= {0x0000,0x0000,0x0000,0x0000,0x0000,0x000,0x010}; //从手的7个极限位置，相对小的一方
//signed short limitMax[7]= {0x0fff,0x0fff,0x0fff,0x0fff,0x0fff,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2018.01.04 for test in lab

extern signed short limitMin[7]; //从手的7个极限位置，相对小的一方
extern signed short limitMax[7]; //从手的7个极限位置，相对大的一方 wangcong2018.01.09
//shortest limitMin[7]= {0x00e0,0x00b0,0x0150,0x0800,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
//longest  limitMax[7]= {0x0f70,0x0f30,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2017.12.25


/////************2015年10月30号更改参数(第1、2、3、5关节)***********************/////////
// Fengjie Qu master paper, P15
extern double Frame[7]; //定义固定连杆长度,单位：毫米 (a)
extern double L0[7]; //直线缸形成为0时的长度
extern double Theta[7]; //定义固定杆和直线缸最小极限夹角（theta）
extern double Conect_rod[7]; //定点至直线缸顶点距离（b）
extern double Trip[7]; //活塞杆行程（L）
extern double Delte[7];
//double Joint_AngleSet[7]={0, 90,- 90, 0, 30, 0, 0x0d3b}; //机械手初始化位姿，弧度1代表57.295度
extern double Joint_AngleSet[7];  //关节角度设定，使用角度
extern double Joint_AngleSet_r[7];  //关节角度设定，使用弧度

extern double Angle_limitMin[7];
extern double Angle_limitMax[7];

// i=4 ??? -Pi/6                  same with matlab
extern double Angle_limitMin_r[7];
extern double Angle_limitMax_r[7];

extern double Joint_Angle[7];

extern double currentJoint[7];  // get current joint values
extern char currentJoint_hex[7];  // get current joint values

extern std::vector<double> jointFromIK;

extern volatile unsigned char NewEnd_Pose;
extern signed char DM[7];   //表征驱动器,为-1时，传感器与关节正方向安装相反

#endif // PARAMETERS_H_INCLUDED

