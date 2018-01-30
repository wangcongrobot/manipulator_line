#ifndef UTILITIES_H_INCLUDED
#define UTILITIES_H_INCLUDED



#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>


#include<errno.h>

#include<string.h>
#include<vector>

#include <math.h>
#include <iostream>
#include <fstream>
#include <iomanip>  // // io 流控制头文件, 主要是一些操纵用法如setw(int n),setprecision(int n),setbase(int   n),setfill(char c)的

using namespace std;

extern int fd;

void Parse( ) ;

void sInit();//从手初始化，开始执行时被调用一次即可
void Run();//周期性执行
void ParseCmd( );
void MoveCtrl();

void SendCtrl();

void Read_from_IK();

void getCurrentJoint();
std::vector<double> IK(double q_current[], double x, double y, double z, int step_num);
void test(double q_current[]);

extern unsigned char uart0_receive_ok;
extern unsigned char len_receive;
extern volatile unsigned char SendEN;
extern volatile int TIMER_FLAG;
extern unsigned char com0RecvBuf[64];
extern unsigned char com0SendBuf[64];///接收与发送缓冲区
extern char   Read_Flag;
extern double Joint_AngleSet[7];
extern double Joint_AngleSet_r[7];
extern double currentJoint[7];
extern char currentJoint_hex[7];

extern struct timeval tv;
extern char  Timer[20];
extern ofstream fout;  //创建一个ofstream流对象
extern string FileName;

extern unsigned short  DIF[6];
extern unsigned char Angle_En;

extern char get_current_joint_flag;

extern std::vector<double> jointFromIK;

extern unsigned char data_record_flag;

#endif // TUTILITIES_H_INCLUDED
