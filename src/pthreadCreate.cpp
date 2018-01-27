#include "pthreadCreate.h"

#include "udp.h"
#include "serial.h"

/*
 是否熟悉POSIX多线程编程技术？如熟悉，编写程序完成如下功能：
  1）有一int型全局变量g_Flag初始值为0；
  2）在主线称中起动线程1，打印“this is thread1”，并将g_Flag设置为1
  3）在主线称中启动线程2，打印“this is thread2”，并将g_Flag设置为2
  4）线程序1需要在线程2退出后才能退出
  5）主线程在检测到g_Flag从1变为2，或者从2变为1的时候退出
   */
/*
 *  when program is started, a single thread is created, called the initial thread or main thread.
 *  Additional threads are created by pthread_create.
 *  So we just need to create two thread in main().
 */

typedef  void*  (*fun)(void*);   //指向函数的指针
int g_Flag=0;
static pthread_mutex_t mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t cond = PTHREAD_COND_INITIALIZER;

void initPthread()
{
    pthread_t tid1, tid2;
    int rc1=0, rc2=0;
    rc2=pthread_create(&tid2, NULL, thread2, NULL);
    usleep(10000);
    if(rc2 != 0)
        printf("pthread_create thread2 error!\n");

    rc1 = pthread_create(&tid1, NULL, thread1, &tid2);
    usleep(10000);
    if(rc1 != 0)
        printf("pthread_create thread1 error!\n");
}

void* thread1(void* arg)
{
    printf("enter thread1\n");
    printf("this is thread1, g_Flag: %d, thread id is %u\n",g_Flag, (unsigned int)pthread_self());

    /*pthread_mutex_lock(&mutex);
    if(g_Flag == 2)
    	pthread_cond_signal(&cond);   //特定信号为真
    g_Flag = 1;
    pthread_mutex_unlock(&mutex);
    */
    while(1)
    {
        //       pthread_mutex_lock(&mutex);    //对多线程更改的数据要进行互斥访问，因串口数据只会在本线程中修改，不需加锁
        ParseCmd();
        //       pthread_mutex_unlock(&mutex);  //

    }
//	pthread_join(*(pthread_t*)arg, NULL);       //等待线程结束，成功返回0，否则返回Exxx（为正数），*(pthread_t*)arg：把arg转换为pthread_t型指针，然后后取出
    printf("leave thread1\n");
    pthread_exit(0);
}

/*
 * thread2() will be execute by thread2, after pthread_create()
 * it will set g_Flag = 2;
 */
void* thread2(void* arg)     //线程2暂时不用
{
    printf("enter thread2\n");
    printf("this is thread2, g_Flag: %d, thread id is %u\n",g_Flag, (unsigned int)pthread_self());  //打印自己的线程ID
    //pthread_mutex_lock(&mutex);
    //if(g_Flag == 1)
    //  pthread_cond_signal(&cond);
    //g_Flag = 2;
    //pthread_mutex_unlock(&mutex);
    while(1)
    {
        receiveFromUdp();
    }
    printf("leave thread2\n");
    pthread_exit(0);
}
