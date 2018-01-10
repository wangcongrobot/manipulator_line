#ifndef CREATEFILE_H_INCLUDED
#define CREATEFILE_H_INCLUDED

#include<stdio.h>
#include<stdlib.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/stat.h>
#include<fcntl.h>
#include<termios.h>
#include<errno.h>
#include<pthread.h>
#include<string.h>
#include <signal.h>
#include <sys/time.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <iomanip>  // // io 流控制头文件, 主要是一些操纵用法如setw(int n),setprecision(int n),setbase(int   n),setfill(char c)的
#include <string>



void createFile();
void dataRecord();

#endif // CREATEFILE_H_INCLUDED
