#ifndef PTHREADCREATE_H_INCLUDED
#define PTHREADCREATE_H_INCLUDED

#include<pthread.h>

#include "utilities.h"

//thread1为指向函数thread1()的指针
void* thread1(void*);

//线程2暂时不用
/*
 * thread2() will be execute by thread2, after pthread_create()
 * it will set g_Flag = 2;
 */
void* thread2(void*);

#endif // PTHREADCREATE_H_INCLUDED
