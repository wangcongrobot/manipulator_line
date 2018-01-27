#include "serial.h"
#include "timer.h"
#include "createFile.h"
#include "motionControl.h"
#include "pthreadCreate.h"
#include "utilities.h"
#include <vector>
#include "ConvertAngle.h"
#include "parameters.h"
#include "Test.h"

///#define DEBUG

//const double Pi = 3.14159265358979;

// unsigned char com0RecvBuf[64];
// unsigned char com0SendBuf[64];  //接收与发送缓冲区
void mainRun();

int main()
{
#ifdef DEBUG
    cout << acos(-1) * 180 / Pi << endl;
    cout << acos(-0.5) * 180 / Pi << endl;
    cout << acos(0) * 180 / Pi << endl;
    cout << acos(0.5) * 180 / Pi << endl;
    cout << acos(1) * 180 / Pi << endl;
    //int qValue[7] = {1507, 103, 2897, 1867, 2932, 1613, 0};
    unsigned short qValue[7] = {0x0200,0x0100,0x0150,0x0750,0x01f0,0x000,0x000};

    double temp[7]= {0};
    //unsigned short input_value[7] = {1,2,3,4,5,6,7};
    double output_value[7] = {0};
    test_func(qValue,output_value);
    for(int i=0; i<7; i++)
    {
        cout << "output: " << output_value[i];
    }
    //char temp = (char*)&(qValue[0]);
    for(int i=0; i<7; i++)
    {
        //cout << hex << qValue[i] << endl;
        cout << setw(5) << dec << qValue[i] << " ";
    }
    cout << endl;
    for(int i=0; i<7; i++)
    {
        cout << setw(5) << hex << qValue[i] << " ";
        //cout << oct << qValue[i] << endl;
    }
    cout << endl;
    //temp[7] = getCurrentJoint1(qValue);
    double test[100]= {0};
    for(int k=0; k<100; k++)
    {
        qValue[0] += 30;
        //temp[7] = getCurrentJoint1(qValue);
        cout << "temp[0] " << temp[0] << endl;
        test[k] = temp[0];
    }
    //temp[7] = getCurrentJoint1(qValue);
    for(int i=0; i<100; i++)
    {
        cout << test[i] << " ";
        //cout << oct << qValue[i] << endl;
    }
    //while(1){sleep(5);}
#endif // DEBUG

    //Test();
    //while(1){};
    //串口初始化
    initSerial();
    //定时器初始化
    initTimer();
    //创建文件记录数据
    createFile();
    //创建线程，串口+UDP
    initPthread();
    //从手初始化，默认Home位置
    sInit();

    // 从手初始化延时，确保运动到Home位置
    for(int i=0; i<5; i++)sleep(1);

    mainRun();
    printf("motion control successfully!\n");


    printf("leave main\n");
    return 0;
}

void mainRun()
{
    //等待获得反馈值
    parse();

    while(get_current_joint_flag==0)  // wait for get current joint
    {
        sleep(1);

        printf("-------------\n");
        continue;
    }

    printf("-----Start motion control!-----\n");

    printf("\n-----Start IK calculation...-----\n");

    jointFromIK = IK(currentJoint, 0, 0, 0.1, 20);

    cout << "Number of jointFromIK: " << (jointFromIK.size() / 6) << endl;
    vector<double>::iterator iter;
    for( iter=jointFromIK.begin(); iter!=(jointFromIK.begin() + 6); iter++)
    {
        cout << *iter * 180 / Pi << " ";
    }
    cout << endl;
    sleep(5);
    int IK_num=jointFromIK.size()/6; //10
    cout << IK_num << endl;

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
        for(int i=0; i<5; i++)usleep(10000);

        getCurrentJoint();
        //SendEN=1;     //发送数据
        printf("Num%d IK send successfully!\n", j);
    }
}


void mainRun1()
{
    //等待获得反馈值
    parse();

    while(get_current_joint_flag==0)  // wait for get current joint
    {
        sleep(1);

        printf("-------------\n");
        continue;
    }
    unsigned short joint_from_slave[7];
    for(int i=0; i<7; i++)
    {
        joint_from_slave[i] = spos->scmdPos[i];  //TODO spos 可能在一直更新？？？
        cout << joint_from_slave[i] << endl;
    }
    double joint_output[7] = {0};
    getCurrentJoint1(joint_from_slave, joint_output);
    printf("-----Start motion control!-----\n");

    printf("\n-----Start IK calculation...-----\n");
    //while(1){sleep(1);}
    //jointFromIK = IK(joint_output, 0, 0, 0.1, 20);
    jointFromIK = IK(currentJoint, 0, 0, 0.1, 20);
    //jointFromIK = IK(currentJoint, 0, 0, 0.3, 10);
    cout << "Number of jointFromIK: " << jointFromIK.size() << endl;
    vector<double>::iterator iter;
    for( iter=jointFromIK.begin(); iter!=(jointFromIK.begin() + 6); iter++)
    {
        cout << *iter * 180 / Pi << endl;
    }
    sleep(5);
    int IK_num=jointFromIK.size()/6; //10
    cout << IK_num << endl;
    //while(1){sleep(1);}
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
        for(int i=0; i<5; i++)usleep(10000);

        getCurrentJoint();
        //SendEN=1;     //发送数据
        printf("Num%d IK send successfully!\n", j);
    }
}


