#ifndef TIMER_H_INCLUDED
#define TIMER_H_INCLUDED

#include <signal.h>
#include <sys/time.h>

#include "utilities.h"

void initTimer();

//定时器函数
void signalHandler(int signo);

#endif // TIMER_H_INCLUDED
