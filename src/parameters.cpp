#include "parameters.h"

signed short limitMin[7]= {0x0000,0x0000,0x0000,0x0000,0x0000,0x000,0x010}; //从手的7个极限位置，相对小的一方
signed short limitMax[7]= {0x0fff,0x0fff,0x0fff,0x0fff,0x0fff,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2018.01.04 for test in lab

///signed short limitMin[7]= {0x0200,0x0100,0x0150,0x0750,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
///signed short limitMax[7]= {0x0fff,0x0ff0,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2018.01.09
//shortest limitMin[7]= {0x00e0,0x00b0,0x0150,0x0800,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
//longest  limitMax[7]= {0x0f70,0x0f30,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2017.12.25

/////************2015年10月30号更改参数(第1、2、3、5关节)***********************/////////
// Fengjie Qu master paper, P15
double Frame[7]= {40.6, 54.8, 54.8, 39.4, 40.6, 56.1, 56.1}; //定义固定连杆长度,单位：毫米 (a)
double L0[7]= {227, 252, 252, 252.1, 227, 227.1, 227.1 }; //直线缸形成为0时的长度
//Theta 3,5,6 ???
//                  30du       24.5     24.5          30
double Theta[7]= {0.525774, 0.429039, 0.429039, 0, 0.525774, 0, 0}; //定义固定杆和直线缸最小极限夹角（theta）
double Conect_rod[7]= {261.2, 300.8, 300.8, 305.5, 261.2, 305.5, 305.5}; //定点至直线缸顶点距离（b）
double Trip[7]= {70, 95,95,95,70,95,95}; //活塞杆行程（L）
double Delte[7]= {0};
//double Joint_AngleSet[7]={0, 90,- 90, 0, 30, 0, 0x0d3b}; //机械手初始化位姿，弧度1代表57.295度
double Joint_AngleSet[7]  = {  0, 90,-60,  0, 90,-170, 0x0c3b};  //关节角度设定，使用角度
double Joint_AngleSet_r[7]= {  0, Pi/2, -Pi/3,  0, Pi/2,-Pi*170.0/180.0, 0x0c3b};  //关节角度设定，使用弧度

double Angle_limitMin[7]= {-60,-30,-90,-90,-30,-170,0x0010 };
double Angle_limitMax[7]= { 60, 90, 30, 90, 90,170,0x0fff };

// i=4 ??? -Pi/6                  same with matlab
double Angle_limitMin_r[7]= {-Pi/3, -Pi/6,-Pi/2,-Pi/2,-Pi/6,-Pi*170.0/180.0,0x0001 };
double Angle_limitMax_r[7]= { Pi/3,  Pi/2, Pi/6, Pi/2, Pi/2, Pi*170.0/180.0,0x0fff };

double Joint_Angle[7]= {0};

double currentJoint[7] = {0};  // get current joint values
char currentJoint_hex[7] = {0};  // get current joint values

std::vector<double> jointFromIK;

volatile unsigned char NewEnd_Pose=0;
signed char DM[7]= {1,1,1,1,1,1,1};   //表征驱动器,为-1时，传感器与关节正方向安装相反
