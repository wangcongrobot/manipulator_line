#include "motionControl.h"
#include "data.h"
#include "serial.h"
#include "utilities.h"
#include "parameters.h"
/*
void sInit()
{
    mCmd.head='M';
    mCmd.END=0x0A0D;   //"\r\n"回车换行
    mCmd.asterisk='*';
    mCmd.Frz=1;   //0:从手激活，1：从手冻结，2：从手锁定，从手关节冻结所有关节都冻结
    mCmd.Hydro=0;       //0：液压关断，1：液压开启
    mCmd.joint[6].pos=0x0c3b;//夹钳主手应该的位置
    mCmd.MoveRatio=1;	  //这个可以改为一个最小的比例系数 beixu
    mCmd.id=0x01;           //正常发送数据
    for(int i=0; i<6; i++)
    {
        mCmd.joint[i].frz=0;        //每个关节解冻
        mCmd.joint[i].mode=0;   //位置模式
    }
    //mCmd.joint[3].frz=0;
    mCmd.joint[6].mode=1;   //夹钳只有速度模式
    ///使机械手回归每次动作的初始点,进行解冻冻结，使下位机自动生成零漂
    mCmd.joint[0].pos=0x0870; //79e
    mCmd.joint[1].pos=0x01b0;
    mCmd.joint[2].pos=0x0f50;
    mCmd.joint[3].pos=0x0a00; //835
    mCmd.joint[4].pos=0x0f40;
    mCmd.joint[5].pos=0x0700;
    mCmd.joint[6].pos=0x0c3b;   //夹钳主手应该的位置c3b

    memcpy(com0SendBuf,&mCmd,sizeof(MCMD));
    writeToSerial(com0SendBuf,sizeof(MCMD));

    //sleep(2);                           //休眠2秒钟，保证机械手回归初始位置

    mCmd.Frz=1;    //冻结
    printf("sInit successfully!\n");
    //getCurrentJoint();
}
*/
void sInit()   //从手初始化，开始执行时被调用一次即可
{

    mCmd.head='M';
    mCmd.END=0x0A0D;   //"\r\n"回车换行
    mCmd.asterisk='*';
    mCmd.Frz=0;   //0:从手激活，1：从手冻结，2：从手锁定，从手关节冻结所有关节都冻结
    mCmd.Hydro=1;       //0：液压关断，1：液压开启
    mCmd.joint[6].pos=0x0c3b;//夹钳主手应该的位置
    mCmd.MoveRatio=1;	  //这个可以改为一个最小的比例系数 beixu
    mCmd.id=0x01;           //正常发送数据
    for(int i=0; i<6; i++)
    {
        mCmd.joint[i].frz=0;    //每个关节解冻
        mCmd.joint[i].mode=0;   //位置模式
    }
    mCmd.joint[3].frz=0;        //每个关节解冻
    mCmd.joint[6].mode=1;   //夹钳只有速度模式
    /**使机械手回归每次动作的初始点,进行解冻冻结，使下位机自动生成零漂***/
    mCmd.joint[0].pos=0x0900; //870
    mCmd.joint[1].pos=0x01b0; //0b0
    mCmd.joint[2].pos=0x0ff0; //fe0
    mCmd.joint[3].pos=0x0b00; //980
    mCmd.joint[4].pos=0x0800; //e00
    mCmd.joint[5].pos=0x0800; //800
    mCmd.joint[6].pos=0x0100; //夹钳主手应该的位置c3b

    // sInit:{0x0870,0x00b0,0x0fe0,0x0b80,0x0f00,0x0800,0x0100}
    //shortest limitMin[7]= {0x00e0,0x00b0,0x0300,0x0800,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
    //longest  limitMax[7]= {0x0f70,0x0f30,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2017.12.25

    memcpy(com0SendBuf,&mCmd,sizeof(MCMD));
    writeToSerial(com0SendBuf,sizeof(MCMD));

    sleep(10);                           //休眠2秒钟，保证机械手回归初始位置

    mCmd.Frz=1;    //冻结
    cout << "sInit successfully!" << endl;
}

void motionControl()
{
    /*
        printf("\n\n-----Start IK calculation...-----\n\n");

        jointFromIK = IK(currentJoint, 0.1, 0, 0, 10);
        cout << "Number of jointFromIK: " << jointFromIK.size() << endl;
        for(int i=0; i<jointFromIK.size(); i++)
        {
            //cout << jointFromIK[i] << endl;

        }

        static int IK_num=0;
        for(int i=0; i<6; i++)
        {

            if(IK_num<jointFromIK.size())
            {
                Joint_AngleSet_r[i] = jointFromIK[IK_num+i];
                IK_num++;
            }
            else
                IK_num=0;
        }
        printf("read from IK successfully!\n");
        if(NewEnd_Pose==0)     //只有当新的末端位置设定时才进行关节角度重新赋值，减轻CPU负担
        {
            SendEN=0;     //正在设定新的关节控制信息，禁止发送数据
            mCmd.Frz=0;
            NewEnd_Pose=0;                   //禁止更新关节信息，直到新的逆解结果更新
            memcpy(com0SendBuf,&mCmd,sizeof(MCMD));    //把新的关节控制信息传给串口数据数组
            printf("motion control successfully!\n");
            SendEN=1;
        }
        printf("\n\n--------Start IK calculation...----------\n\n");

        jointFromIK = IK(currentJoint, 0.1, 0, 0, 10);
        cout << "Number of jointFromIK: " << jointFromIK.size() << endl;
        for(int i=0; i<jointFromIK.size(); i++)
        {
            //cout << jointFromIK[i] << endl;

        }


        static int IK_num=0;
        for(int i=0; i<6; i++)
        {

            if(IK_num<jointFromIK.size())
            {
                Joint_AngleSet_r[i] = jointFromIK[IK_num+i];
                IK_num++;
            }
            else
                IK_num=0;
            //return 0;
            //exit (0);

            cout << "Joint_AngleSet_r" << i << " -- " << Joint_AngleSet_r[i] << endl;

        }
        printf("read from IK successfully!\n");
        if(NewEnd_Pose==0)     //只有当新的末端位置设定时才进行关节角度重新赋值，减轻CPU负担
        {
            SendEN=0;     //正在设定新的关节控制信息，禁止发送数据
            mCmd.Frz=0;
            NewEnd_Pose=0;                   //禁止更新关节信息，直到新的逆解结果更新
            memcpy(com0SendBuf,&mCmd,sizeof(MCMD));    //把新的关节控制信息传给串口数据数组
            printf("motion control successfully!\n");
            SendEN=1;
        }
    */
}

void sendCtrl()
{
    /*
    TIMER_FLAG++;
    if(TIMER_FLAG==40)
    {
        TIMER_FLAG=0;
        Angle_En=1;      //记录500毫秒时间标志位
    }
    */
    if(SendEN==1)
    {
        writeToSerial(com0SendBuf,24);
        printf("\nSendCtrl to serial successfully!\n");
    }
    //printf("\nSendCtrl successfully!\n");
    //printf("sendCtrl Hello\n");
}
void Run()
{
    //if(receive_flag>0)Parse();
    motionControl();
}

// 应该是调用一次该函数，返回当前关节值反馈
// 得到当前的关节值   逻辑关系有问题？？
void parse()
{
    // 接收正确数据，并且反馈值是角度实际值
    // 阻塞函数
    while(1)
    {
        if(uart0_receive_ok == 1) // 接收成功
        {
            //printf("\nuart0_receive_ok == 1\n");
            if(com0RecvBuf[1] == 0x00) // 关节实际值
            {
                SendEN=0;
                printf("\ncom0RecvBuf[1] == 0x00\n");
                // (char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
                // 关节值
                spos=(SPOS*)&(com0RecvBuf[0]);
                getCurrentJoint(); /// 关节反馈进行解析，得到关节角
                printf("getCurrentJoint() successfully!\n");
                //打印该关节值
                unsigned char *p;
                for(p=com0RecvBuf; p<(com0RecvBuf+24); p++)
                {
                    printf("%x ",*p);
                }
                printf("\n");

                uart0_receive_ok=0;
                Bit.com0Recved=0; //若校验成功，需要解析完之后允许串口接收允许再次接收数据
                printf("\nparse() successfully!\n");
                break;
            }
            SendEN=1;
        }
    }
}


// FK
void test_getCurrentJoint()
{
    double c;
    //double q0[7] = {1831,293,3437,2512,3630,1953,0};
    double q0[7] = {2048,10,4000,2448,4000,2048,0};
    for(int i=0; i<7; i++)
    {
        cout << q0[i] << " ";
    }
    cout << endl;

    for(int i=0; i<7; i++)                      //关节角度更新，只有前六个关节有反馈
    {
        if((i==0) || (i==1) ||(i==2) || (i==4))
        {
            //设定关节角时，直线缸长度的平方 (余弦定理）
            //Delte[i]=pow(Frame[i], 2)+pow(Conect_rod[i], 2) -2*Frame[i]*Conect_rod[i]*cos(Theta[i]+Joint_Angle[i]);
            //cout << Delte[i] << endl;
            //convert to hex
            //mCmd.joint[i].pos=(ushort) ((Trip[i]-sqrt( Delte[i])+L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) ;
            //cout << "mCmd.joint" << i << "  " << ((Trip[i]-sqrt( Delte[i])+L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) << endl;
            //cout << mCmd.joint[i].pos << endl;
            //cout << spos->scmdPos[i] << endl;
            //cout << (int)(spos->scmdPos[i]-limitMin[i]) << endl;
            cout << (double)(limitMax[i]-limitMin[i]) << endl;
            cout << ((double)(q0[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i])) << endl;
            cout << (double)((q0[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i]))*Trip[i] << endl;
            c = L0[i]+((double)(q0[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];
            cout << "c" << i << "= " << c << endl;
            cout << pow(c,2) << endl;
            cout << pow(Frame[i],2) << endl;
            cout << pow(Conect_rod[i],2) << endl;
            cout << acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) << endl;
            //0~Pi
            currentJoint[i] = acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) - Theta[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
            cout << "currentJoint " << i << "  :" << currentJoint[i] << endl;
            currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
        else if(i==3)
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/Pi+limitMin[i]);
            cout << (double)(q0[i]-limitMin[i]) << endl;
            cout << ((double)(limitMax[i]-limitMin[i])) << endl;
            cout << ((double)(q0[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i])) << endl;
            currentJoint[i] = ((double)(q0[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Pi;
            cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
            currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
        else if (i==6)
        {

            //mCmd.joint[i].pos= Joint_Angle[6];   //以0x0c3b为基准，大于这个值闭合，小于张开
            currentJoint[i]=q0[i];
            cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
        }
        else     // (i==5)
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/(5.93411945678)+limitMin[i]);     //i=5时
            cout << ((double)(q0[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i])) << endl;
            cout << ((double)(q0[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2) << endl;

            currentJoint[i]=((double)(q0[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2);
            cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
            currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
        if(currentJoint[i] - Angle_limitMin_r[i] <= EPSINON)
        {
            currentJoint[i] =Angle_limitMin_r[i];
            cout << "currentJoint" << " < " << "Angle_limitMin_r i=" << i << endl;
        }
        else if(currentJoint[i] - Angle_limitMax_r[i] >= EPSINON)
        {
            currentJoint[i]=Angle_limitMax_r[i];
            cout << "currentJoint" << " > " << "Angle_limitMax_r i=" << i << endl;
        }
    }

    // convert
    //currentJoint[0] -= Pi / 3;
    //currentJoint[1] -= Pi / 6;
    //currentJoint[2] -= Pi / 2;
    //currentJoint[3] -= Pi / 2;
    //currentJoint[4] -= Pi / 2; //Pi/6 ???
    //currentJoint[5] -= Pi * 170 /180; //第六个关节340连续旋转
    //currentJoint[6]  = currentJoint[6];

    for(int i=0; i<6; i++)
    {
        cout << "actual currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        //cout << "actual currentJoint" << i << "  " << currentJoint[i] << endl;
    }
    printf("\n\n********get current joint values successfully!**********\n\n");
    get_current_joint_flag=1;
    //printf("get_current_joint_flag=1\n");
    test(currentJoint);
}
