#include "motionControl.h"
#include "data.h"
#include "serial.h"
#include "utilities.h"

const double Pi = 3.14159265358979;
const double EPSINON = 1e-5;

// unsigned short   sAct_DIF[6];
unsigned short  Joint_DIF[6]= {0x0010,0x0010,0x0010,0x0010,0x0010,0x0010}; //单关节到达设定位置的差值
unsigned char  Joint_flag=0;    //单关节一次不动标志
unsigned char  ARM_flag0=1;   //机械臂一次不动标志
//unsigned char  ARM_flag1=0;   //机械臂五次不动标志
//signed short limitMin[7]= {0x0000,0x0000,0x0000,0x0000,0x0000,0x000,0x010}; //从手的7个极限位置，相对小的一方
//signed short limitMax[7]= {0x0fff,0x0fff,0x0fff,0x0fff,0x0fff,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2018.01.04 for test in lab

signed short limitMin[7]= {0x0200,0x0100,0x0150,0x0750,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
signed short limitMax[7]= {0x0fff,0x0ff0,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2018.01.09
//shortest limitMin[7]= {0x00e0,0x00b0,0x0150,0x0800,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
//longest  limitMax[7]= {0x0f70,0x0f30,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2017.12.25


/////************2015年10月30号更改参数(第1、2、3、5关节)***********************/////////
// Fengjie Qu master paper, P15
double Frame[7]= {40.6, 54.8, 54.8, 39.4, 40.6, 56.1, 56.1}; //定义固定连杆长度,单位：毫米 (a)
double L0[7]= {227, 252, 252, 252.1, 227, 227.1, 227.1 }; //直线缸形成为0时的长度
double Theta[7]= {0.5258, 0.4290, 0.4290, 0.1308, 0.5258, 0.1308, 0.1308}; //定义固定杆和直线缸最小极限夹角（theta）
double Conect_rod[7]= {261.2, 300.8, 300.8, 305.5, 261.2, 305.5, 305.5}; //定点至直线缸顶点距离（b）
double Trip[7]= {70, 95,95,95,70,95,95}; //活塞杆行程（L）
double Delte[7]= {0};
//double Joint_AngleSet[7]={0, 90,- 90, 0, 30, 0, 0x0d3b}; //机械手初始化位姿，弧度1代表57.295度
double Joint_AngleSet[7]  = {  0, 90,-60,  0, 90,-170, 0x0c3b};  //关节角度设定，使用角度
double Joint_AngleSet_r[7]= {  0, Pi/2, -Pi/3,  0, Pi/2,-Pi*170.0/180.0, 0x0c3b};  //关节角度设定，使用弧度

double Angle_limitMin[7]= {-60,-30,-90,-90,-30,-170,0x0010 };
double Angle_limitMax[7]= { 60, 90, 30, 90, 90,170,0x0fff };

// i=4 ??? -Pi/6                  same with matlab
double Angle_limitMin_r[7]= {-Pi/3, -Pi/6,-Pi/2,-Pi/2,-Pi/6,-Pi*170.0/180.0,0x0001 };
double Angle_limitMax_r[7]= { Pi/3,  Pi/2, Pi/6, Pi/2, Pi/2, Pi*170.0/180.0,0x0fff };

double Joint_Angle[7]= {0};

double currentJoint[7] = {0};  // get current joint values
char currentJoint_hex[7] = {0};  // get current joint values

std::vector<double> jointFromIK;

volatile unsigned char NewEnd_Pose=0;
signed char DM[7]= {1,1,1,1,1,1,1};   //表征驱动器,为-1时，传感器与关节正方向安装相反
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
void sInit()//从手初始化，开始执行时被调用一次即可
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
    mCmd.joint[4].pos=0x0f00; //e00
    mCmd.joint[5].pos=0x0800; //800
    mCmd.joint[6].pos=0x0100;   //夹钳主手应该的位置c3b

    // sInit:{0x0870,0x00b0,0x0fe0,0x0b80,0x0f00,0x0800,0x0100}
    //shortest limitMin[7]= {0x00e0,0x00b0,0x0300,0x0800,0x01f0,0x000,0x000}; //从手的7个极限位置，相对小的一方
    //longest  limitMax[7]= {0x0f70,0x0f30,0x0fe0,0x0f60,0x0ff0,0x0fff,0x0fff}; //从手的7个极限位置，相对大的一方 wangcong2017.12.25

    memcpy(com0SendBuf,&mCmd,sizeof(MCMD));
    writeToSerial(com0SendBuf,sizeof(MCMD)) ;

    sleep(10);                           //休眠2秒钟，保证机械手回归初始位置

    mCmd.Frz=1;    //冻结

}

// FK
void getCurrentJoint()
{
    double c;
    //double q0[7] = {1831,293,3437,2512,3630,1953,0};
    for(int i=0; i<7; i++)
    {
        cout << spos->scmdPos[i] << " ";
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
            //cout << (double)(limitMax[i]-limitMin[i]) << endl;
            //cout << ((double)(spos->scmdPos[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i])) << endl;
            //cout << (double)((spos->scmdPos[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i]))*Trip[i] << endl;
            // by zyx
            //c = Trip[i] + L0[i] - ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];
            // by wangcong
            c = L0[i]+((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];

            cout << "c" << i << "= " << c << endl;
            //cout << pow(c,2) << endl;
            //cout << pow(Frame[i],2) << endl;
            //cout << pow(Conect_rod[i],2) << endl;
            cout << acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) << endl;
            //0~Pi
            currentJoint[i] = acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) - Theta[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
            //cout << "currentJoint " << i << "  :" << currentJoint[i] << endl;
            //currentJoint[i] += Angle_limitMin_r[i];
            //cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
        else if(i==3)
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/Pi+limitMin[i]);
            currentJoint[i] = ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Pi;
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
            //currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
        else if (i==6)
        {

            //mCmd.joint[i].pos= Joint_Angle[6];   //以0x0c3b为基准，大于这个值闭合，小于张开
            currentJoint[i]=spos->scmdPos[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
        else // (i==5)
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/(5.93411945678)+limitMin[i]);     //i=5时
            //cout << ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i])) << endl;
            //cout << ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2) << endl;

            currentJoint[i]=((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2);
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
            //currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180/Pi << endl;
        }
    }
    // convert -Pi/3+0.276954082, Pi/2,-Pi/2,-Pi/2,-Pi/2,-Pi*170.0/180.0
    currentJoint[0] += -Pi/4;
    currentJoint[1]  =  Pi/2-currentJoint[1]-0.22677;
    currentJoint[2]  =  -currentJoint[2] + 0.366333333; // 21du
    currentJoint[3]  = 0.738432056-currentJoint[3]; //50
    currentJoint[4] += -Pi/2; //Pi/6 ???
    currentJoint[5] += Angle_limitMin_r[5]; //第六个关节340连续旋转
    currentJoint[6]  = currentJoint[6];
    for(int i=0; i<6; i++)
    {
        cout << "currentJoint += Angle_limitMin_r[i]" << i << "  " << currentJoint[i]*180/Pi << endl;

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

void AngleConvert()     // 关节角度更新函数
{
    for(int i=0; i<6; i++)
    {
        //cout << Joint_AngleSet_r[i] *180/Pi<< endl;
    }
    for(int i=0; i<6; i++)
    {
        //cout << "limitMin " << i << " " << limitMin[i] << endl;
        //cout << "limitMax " << i << " " << limitMax[i] << endl;
        if ((Angle_limitMin_r[i]-Joint_AngleSet_r[i])>=EPSINON)
        {
            Joint_AngleSet_r[i]=Angle_limitMin_r[i];
            cout << "Joint_AngleSet_r " << Joint_AngleSet_r[i]*180/Pi << " <= "
                 << "Angle_limitMin_r " << Angle_limitMin_r[i]*180/Pi << " i=" << i << endl;
        }
        if (Joint_AngleSet_r[i]-Angle_limitMax_r[i]>=EPSINON)
        {
            Joint_AngleSet_r[i]=Angle_limitMax_r[i];
            cout << "Joint_AngleSet_r " << Joint_AngleSet_r[i]*180/Pi << " >= "
                 << "Angle_limitMax_r " << Angle_limitMax_r[i]*180/Pi << " i=" << i << endl;
        }
        //cout << "Joint_AngleSet_r:" << Joint_AngleSet_r[i] << endl; // read from the data trajectory file
    }
    // Pi为math.h中定义的圆周率，角度转换成弧度
    // 弧度-->feifushu
    for(int i=0; i<6; i++)
    {
        Joint_Angle[i] = Joint_AngleSet_r[i] * DM[i] - Angle_limitMin_r[i];
        cout << "(Joint_AngleSet_r[i] * DM[i])" << (Joint_AngleSet_r[i] * DM[i] - Angle_limitMin_r[i]) << endl;
        cout << "(Joint_AngleSet_r[i] * DM[i] - Angle_limitMin_r[i])" << (Joint_AngleSet_r[i] * DM[i] - Angle_limitMin_r[i]) << endl;
        cout << "(Joint_AngleSet_r[i] * DM[i] - Angle_limitMin_r[i])" << (Joint_AngleSet_r[i] * DM[i] - Angle_limitMin_r[i]) << endl;
    }
    /*
    Joint_Angle[0] +=  Pi/4;
    Joint_Angle[1]  =  Pi/2-Joint_AngleSet_r[1]-0.22677;
    Joint_Angle[2]  =  0.366333333-Joint_AngleSet_r[2]; // 21du
    Joint_Angle[3]  = 0.738432056-Joint_AngleSet_r[3]; //50
    Joint_Angle[4] += Pi/2; //Pi/6 ???
    Joint_Angle[5] -= Angle_limitMin_r[5]; //第六个关节340连续旋转
    Joint_Angle[6]  = Joint_AngleSet_r[6];
*/
    //currentJoint[0]  = currentJoint[0]-Pi/4;
    //currentJoint[1]  =  Pi/2-currentJoint[1];
    //currentJoint[2]  =  -currentJoint[2] + 0.366333333; // 21du
    //currentJoint[3]  = 0.738432056-currentJoint[3]; //50
    //currentJoint[4]  = currentJoint[4]-Pi/2; //Pi/6 ???
    //currentJoint[5]  = currentJoint[5]+Angle_limitMin_r[5]; //第六个关节340连续旋转
    //currentJoint[6]  = currentJoint[6];
    //Joint_Angle[0]=Joint_AngleSet_r[0]*DM[0]+Pi /3;
    //Joint_Angle[1]=Joint_AngleSet_r[1]*DM[1]+Pi /6;
    //Joint_Angle[2]=Joint_AngleSet_r[2]*DM[2]+Pi /2;
    //Joint_Angle[3]=Joint_AngleSet_r[3]*DM[3]+Pi /2;
    //Joint_Angle[4]=Joint_AngleSet_r[4]*DM[4]+Pi /2;
    //Joint_Angle[5]=Joint_AngleSet_r[5]*DM[5]+Pi * 170 /180; //   170               //第六个关节340连续旋转
    //Joint_Angle[6]=Joint_AngleSet_r[6];      //Joint_AngleSet[6]以0x0c3b为基准，小于这个值张开，大于于闭合

    //double c;
    for(int i=0; i<7; i++)
    {
        cout << "Joint_Angle[i]" << " " << Joint_Angle[i]*180/Pi << endl;
    }
    for(int i=0; i<7; i++) //关节角度更新，只有前六个关节有反馈
    {
        mCmd.joint[i].frz=0;      //保证每个关节更新的时候，处于解冻状态
        if((i==0) || (i==1) || (i==2) || (i==4))
        {
            //设定关节角时，直线缸长度的平方 (余弦定理）
            // TODO (Theta[i]+Joint_Angle[i])
            Delte[i]=pow(Frame[i], 2)+pow(Conect_rod[i], 2) -2*Frame[i]*Conect_rod[i]*cos(Theta[i]+Joint_Angle[i]);
            //cout << Delte[i] << endl;
            //convert to hex
            //cout << (sqrt(Delte[i])-L0[i])*(limitMax[i]-limitMin[i]) << endl;
            //cout << (sqrt(Delte[i])-L0[i])*(limitMax[i]-limitMin[i])/Trip[i] << endl;
            //cout << (ushort) ((sqrt(Delte[i])-L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) << endl;
            // by wangcong
            mCmd.joint[i].pos=(ushort) ((sqrt(Delte[i])-L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) ;
            //by zyx
            //mCmd.joint[i].pos=(ushort) ((Trip[i]-sqrt( Delte[i])+L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) ;
            //cout << "mCmd.joint" << i << "  " << ((Trip[i]-sqrt( Delte[i])+L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) << endl;
            //cout << mCmd.joint[i].pos << endl;

            ///c = Trip[i]+L0[i]-(mCmd.joint[i].pos-limitMin[i])*Trip[i]/(limitMax[i]-limitMin[i]);
            //cout << c << endl;
            //cout << pow(c,2) << endl;
            //0~Pi
            ///currentJoint[i] = acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) - Theta[i];
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
        }
        else if(i==3)
        {
            mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/Pi+limitMin[i]);

            ///currentJoint[i] = (mCmd.joint[i].pos-limitMin[i])*Pi/(limitMax[i]-limitMin[i]);
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
        }
        else if (i==6)
        {

            mCmd.joint[i].pos= Joint_Angle[6];   //以0x0c3b为基准，大于这个值闭合，小于张开

            ///currentJoint[i]=mCmd.joint[i].pos;
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
        }
        else  //i=5时
        {
            mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/(170.0/180.0*Pi*2)+limitMin[i]);

            ///currentJoint[i]=(mCmd.joint[i].pos-limitMin[i])*5.93411945678/(limitMax[i]-limitMin[i]);
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
        }
        if(mCmd.joint[i].pos<limitMin[i])
        {
            mCmd.joint[i].pos=limitMin[i];
        }
        else if(mCmd.joint[i].pos>limitMax[i])
        {
            mCmd.joint[i].pos=limitMax[i];
        }

        cout << "mCmd.joint[i].pos" << "  " <<mCmd.joint[i].pos << endl;

    }
    printf("AngleConvert successfully!\n");
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
        writeToSerial(com0SendBuf,24) ;
        printf("\nSendCtrl to serial successfully!\n");
    }
    //printf("\nSendCtrl successfully!\n");
    //printf("sendCtrl Hello\n");
}
void Run()
{

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
        else // (i==5)
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
