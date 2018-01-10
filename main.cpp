#include "serial.h"
#include "timer.h"
#include "createFile.h"
#include "motionControl.h"
#include "pthreadCreate.h"
#include "utilities.h"
#include <vector>

const double Pi = 3.14159265358979;

unsigned char com0RecvBuf[64];
unsigned char com0SendBuf[64];  //接收与发送缓冲区

int main()
{
    initSerial();
    initTimer();
    createFile();

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

    printf("enter main\n");

    sInit();
    for(int i=0;i<5;i++)sleep(1);
    //while(1){sleep(5);}
    //sleep(5);

    // call for getting current joint
    //getCurrentJoint();
    //while(1){sleep(1);}
    parse();

    while(get_current_joint_flag==0)  // wait for get current joint
    {
        sleep(1);

        printf("-------------\n");
        continue;
    }

    printf("-----Start motion control!-----\n");

    printf("\n-----Start IK calculation...-----\n");
    //while(1){sleep(1);}
    jointFromIK = IK(currentJoint, 0, 0, 0.2, 50);
    cout << "Number of jointFromIK: " << jointFromIK.size() << endl;
    vector<double>::iterator iter;
    for( iter=jointFromIK.begin(); iter!=(jointFromIK.begin() + 6); iter++)
    {
        cout << *iter*180/Pi << endl;
    }
    sleep(5);
    int IK_num=jointFromIK.size()/6; //10
    cout << IK_num << endl;
    //return 0;
    for(int j=0; j<IK_num; j++)
    {
        SendEN=0;     //正在设定新的关节控制信息，禁止发送数据
        int k=0;
        for(iter=(jointFromIK.begin()+j*6); iter!=(jointFromIK.begin()+j*6 +6); iter++)
        {
            Joint_AngleSet_r[k] = *iter;
            cout << "Joint_AngleSet_r" << k << " -- " << Joint_AngleSet_r[k] *180/Pi<< endl;
            k++;
        }
        usleep(50000);
        mCmd.Frz=0;
        AngleConvert();
        //NewEnd_Pose=0; //禁止更新关节信息，直到新的逆解结果更新
        memcpy(com0SendBuf,&mCmd,sizeof(MCMD));    //把新的关节控制信息传给串口数据数组
        writeToSerial(com0SendBuf,24);
        //SendEN=1;     //发送数据
        printf("Num%d IK send successfully!\n", j);
    }

    printf("motion control successfully!\n");
    parse();
    sleep(10);

printf("leave main\n");
return 0;
}


