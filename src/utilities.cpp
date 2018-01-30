#include "utilities.h"

unsigned short  DIF[6]= {0};
unsigned char Angle_En=1;

unsigned char data_record_flag=0;

char Timer[20] = {0};
unsigned int sum = 0;

struct timeval tv;
ofstream fout;  //创建一个ofstream流对象
string FileName;

char get_current_joint_flag=0;
