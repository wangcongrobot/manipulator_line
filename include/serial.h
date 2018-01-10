#ifndef SERIAL_H_INCLUDED
#define SERIAL_H_INCLUDED

#include<termios.h>
#include<fcntl.h>

#include "data.h"
#include "createFile.h"
#include "utilities.h"

#define BAUDRATE        19200
#define UART_DEVICE     "/dev/ttyUSB0"
#define FALSE  -1
#define TRUE   0

extern unsigned char uart0_receive_ok;

////////////////////////////////////////////////////////////////////////////////
/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*//////////////////////////////////////////////////////////////////////////////
int setParity(int fd,int databits,int stopbits,int parity);

void initSerial();

//把校验位加进去，使用指针强制类型转换方式
void writeToSerial(unsigned char *buf,unsigned char len);

//接收数据函数
void receiveFromSerial(unsigned char length);

//校验接收数据函数
unsigned char CheckRecvData();

//判断接收到一条完整的数据条
void ParseCmd();


#endif // SERIAL_H_INCLUDED
