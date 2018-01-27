#include "utilities.h"

unsigned short  DIF[6]= {0};
unsigned char Angle_En=1;

char Timer[20] = {0};
unsigned int sum = 0;

struct timeval tv;
ofstream fout;  //创建一个ofstream流对象
string FileName;

char get_current_joint_flag=0;

unsigned char com0RecvBuf[64]={0};
unsigned char com0SendBuf[64]={0};  //接收与发送缓冲区
