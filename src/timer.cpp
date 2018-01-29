#include "timer.h"
#include "motionControl.h"

void initTimer()
{
    signal(SIGALRM, signalHandler); 	//SIGALRM 信号:定时器产生的信号。sigaql(*,*)两个参数，一个代表信号类型，一个带个处理方式（函数）

    struct itimerval new_value, old_value;
    new_value.it_value.tv_sec = 0;
    new_value.it_value.tv_usec = 10;                 //10微秒后启动定时器
    new_value.it_interval.tv_sec =0;
    new_value.it_interval.tv_usec = 500000;    //启动定时器后每隔50毫秒(50000)启动一次
    setitimer(ITIMER_REAL, &new_value, &old_value);
    printf("Timer initial successfully!\n");
}

void signalHandler(int signo)
{
    switch (signo)
    {
    case SIGALRM:
    {
        //printf("Caught the SIGALRM signal! %d\n", i);
        sendCtrl();
        //printf("timer: 50ms\n");
    }

    break;

    }
}
