#include "ConvertAngle.h"

#include "motionControl.h"
#include "data.h"
#include "serial.h"
#include "utilities.h"
#include "parameters.h"


//test
void test_func(unsigned short const *input, double *output)
{
    for(int i=0; i<7; i++)
        output[i] = input[i] * 10;
}


// FK
// 输入当前关节反馈值，输出各关节关节角
void getCurrentJoint1(unsigned short const *joint_input, double *joint_output)
{
    //double joint_output[7]={0};
    double c; // 直线缸长度
    cout << "q from feedback: " ;
    for(int i=0; i<7; i++)
    {
        cout << joint_input[i] << " ";
    }
    cout << endl;

    //关节角度转换
    for(int i=0; i<7; i++)
    {
        //直线缸0、1、2、4，余弦定理求解
        if((i==0) || (i==1) ||(i==2) || (i==4))
        {
            //设定关节角时，直线缸长度的平方 (余弦定理）
            //Delte[i]=pow(Frame[i], 2)+pow(Conect_rod[i], 2) -2*Frame[i]*Conect_rod[i]*cos(Theta[i]+Joint_Angle[i]);
            //cout << Delte[i] << endl;
            //convert to hex
            //mCmd.joint[i].pos=(ushort) ((Trip[i]-sqrt( Delte[i])+L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) ;
            //cout << "mCmd.joint" << i << "  " << ((Trip[i]-sqrt( Delte[i])+L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) << endl;
            //cout << mCmd.joint[i].pos << endl;
            cout << "joint_input "  << joint_input[i] << endl;
            cout << "(joint_input[i]-limitMin[i]) "  << (double)(joint_input[i]-limitMin[i]) << endl;
            cout << "(limitMax[i]-limitMin[i]) "  << (double)(limitMax[i]-limitMin[i]) << endl;
            cout << "(joint_input[i]-limitMin[i])/(limitMax[i]-limitMin[i]) "  << ((double)(joint_input[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i])) << endl;
            cout << "((joint_input[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i]))*Trip[i] "  << (double)((joint_input[i]-limitMin[i])/(double)(limitMax[i]-limitMin[i]))*Trip[i] << endl;
            // by zyx
            //c = Trip[i] + L0[i] - ((double)(joint_input[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];
            // by wangcong
            // 直线缸长度 = 最短（L0）+ 伸出部分
            c = L0[i]+((double)(joint_input[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];

            cout << "c" << i << "= " << c << endl;
            //cout << pow(c,2) << endl;
            //cout << pow(Frame[i],2) << endl;
            //cout << pow(Conect_rod[i],2) << endl;
            cout << "acos "  << acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) << endl;
            //0~Pi
            joint_output[i] = acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) - Theta[i];
            cout << "joint_output" << i << "  " << joint_output[i]*180.0/Pi << endl;
            //cout << "joint_output " << i << "  :" << joint_output[i] << endl;
            //joint_output[i] += Angle_limitMin_r[i];
            //cout << "joint_output" << i << "  " << joint_output[i]*180.0/Pi << endl;
        }
        else if(i==3) // 旋转关节
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/Pi+limitMin[i]);
            joint_output[i] = ((double)(joint_input[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Pi;
            //cout << "joint_output" << i << "  " << joint_output[i] << endl;
            //joint_output[i] += Angle_limitMin_r[i];
            cout << "joint_output" << i << "  " << joint_output[i]*180.0/Pi << endl;
        }
        else if (i==6)
        {
            //mCmd.joint[i].pos= Joint_Angle[6];   //以0x0c3b为基准，大于这个值闭合，小于张开
            joint_output[i]=joint_input[i];
            cout << "joint_output" << i << "  " << joint_output[i]*180.0/Pi << endl;
        }
        else // i==5 旋转关节
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/(5.93411945678)+limitMin[i]);     //i=5时
            //cout << ((double)(joint_input[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i])) << endl;
            //cout << ((double)(joint_input[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2) << endl;

            joint_output[i]=((double)(joint_input[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2);
            //cout << "joint_output" << i << "  " << joint_output[i] << endl;
            //joint_output[i] += Angle_limitMin_r[i];
            cout << "joint_output" << i << "  " << joint_output[i]*180.0/Pi << endl;
        }
    }
    // convert -Pi/3+0.276954082, Pi/2,-Pi/2,-Pi/2,-Pi/2,-Pi*170.0/180.0
    /*
    joint_output[0] += -Pi/4;
    joint_output[1]  =  Pi/2-joint_output[1]-0.22677;
    joint_output[2]  =  -joint_output[2] + 0.366333333; // 21du
    joint_output[3]  = 0.738432056-joint_output[3]; //50
    joint_output[4] += -Pi/2; //Pi/6 ???
    joint_output[5] += Angle_limitMin_r[5]; //第六个关节340连续旋转
    joint_output[6]  = joint_output[6];
    */
    joint_output[0] += -Pi/3;
    joint_output[1] += -Pi/6;
    joint_output[2] += -Pi/2;
    joint_output[3] += -Pi/2;
    joint_output[4] += -Pi/2;
    joint_output[5] += -Pi*170.0/180.0; //第六个关节340连续旋转
    //joint_output[6] += ;
    for(int i=0; i<6; i++)
    {
        cout << "joint_output += Angle_limitMin_r[i]" << i << "  " << joint_output[i]*180.0/Pi << endl;

        if(joint_output[i] - Angle_limitMin_r[i] <= EPSINON)
        {
            joint_output[i] =Angle_limitMin_r[i];
            cout << "joint_output" << " < " << "Angle_limitMin_r i=" << i << endl;
        }
        else if(joint_output[i] - Angle_limitMax_r[i] >= EPSINON)
        {
            joint_output[i]=Angle_limitMax_r[i];
            cout << "joint_output" << " > " << "Angle_limitMax_r i=" << i << endl;
        }
    }

    for(int i=0; i<6; i++)
    {
        cout << "actual joint_output" << i << "  " << joint_output[i]*180.0/Pi << endl;
        //cout << "actual joint_output" << i << "  " << joint_output[i] << endl;
    }
    printf("\n\n********get current joint values successfully!**********\n\n");
    get_current_joint_flag=1;
    //printf("get_current_joint_flag=1\n");
    //test(joint_output);
    //return joint_output[7];
}
void getCurrentJoint()
{
    double c;
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
            c = Trip[i] + L0[i] - ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];
            // by wangcong
            //c = L0[i]+((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Trip[i];

            cout << "c" << i << "= " << c << endl;
            //cout << pow(c,2) << endl;
            //cout << pow(Frame[i],2) << endl;
            //cout << pow(Conect_rod[i],2) << endl;
            cout << acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) << endl;
            //0~Pi
            currentJoint[i] = acos((pow(Frame[i],2)+pow(Conect_rod[i],2)-pow(c,2))/(2*Frame[i]*Conect_rod[i])) - Theta[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180.0/Pi << endl;
            //cout << "currentJoint " << i << "  :" << currentJoint[i] << endl;
            //currentJoint[i] += Angle_limitMin_r[i];
            //cout << "currentJoint" << i << "  " << currentJoint[i]*180.0/Pi << endl;
        }
        else if(i==3)
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/Pi+limitMin[i]);
            currentJoint[i] = ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*Pi;
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
            //currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180.0/Pi << endl;
        }
        else if (i==6)
        {

            //mCmd.joint[i].pos= Joint_Angle[6];   //以0x0c3b为基准，大于这个值闭合，小于张开
            currentJoint[i]=spos->scmdPos[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180.0/Pi << endl;
        }
        else // (i==5)
        {
            //mCmd.joint[i].pos=(ushort)(Joint_Angle[i]*(limitMax[i]-limitMin[i])/(5.93411945678)+limitMin[i]);     //i=5时
            //cout << ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i])) << endl;
            //cout << ((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2) << endl;

            currentJoint[i]=((double)(spos->scmdPos[i]-limitMin[i]))/((double)(limitMax[i]-limitMin[i]))*(170.0/180.0*Pi*2);
            //cout << "currentJoint" << i << "  " << currentJoint[i] << endl;
            //currentJoint[i] += Angle_limitMin_r[i];
            cout << "currentJoint" << i << "  " << currentJoint[i]*180.0/Pi << endl;
        }
    }
    /*
    // convert -Pi/3+0.276954082, Pi/2,-Pi/2,-Pi/2,-Pi/2,-Pi*170.0/180.0
    currentJoint[0] += -Pi/4;
    currentJoint[1]  =  Pi/2-currentJoint[1]-0.22677;
    currentJoint[2]  =  -currentJoint[2] + 0.366333333; // 21du
    currentJoint[3]  = 0.738432056-currentJoint[3]; //50
    currentJoint[4] += -Pi/2; //Pi/6 ???
    currentJoint[5] += Angle_limitMin_r[5]; //第六个关节340连续旋转
    //currentJoint[6]  = currentJoint[6];
    */
    currentJoint[0] += -Pi/3;
    currentJoint[1] += -Pi/6;
    currentJoint[2] += -Pi/2;
    currentJoint[3] += -Pi/2;
    currentJoint[4] += -Pi/2;
    currentJoint[5] += -Pi*170.0/180.0; //第六个关节340连续旋转

    for(int i=0; i<6; i++)
    {
        cout << "currentJoint += Angle_limitMin_r[i]" << i << "  " << currentJoint[i]*180.0/Pi << endl;

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
        cout << "actual currentJoint" << i << "  " << currentJoint[i]*180.0/Pi << endl;
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
        //cout << Joint_AngleSet_r[i] *180.0/Pi<< endl;
    }
    for(int i=0; i<6; i++)
    {
        //cout << "limitMin " << i << " " << limitMin[i] << endl;
        //cout << "limitMax " << i << " " << limitMax[i] << endl;
        if ((Angle_limitMin_r[i]-Joint_AngleSet_r[i])>=EPSINON)
        {
            Joint_AngleSet_r[i]=Angle_limitMin_r[i];
            cout << "Joint_AngleSet_r " << Joint_AngleSet_r[i]*180.0/Pi << " <= "
                 << "Angle_limitMin_r " << Angle_limitMin_r[i]*180.0/Pi << " i=" << i << endl;
        }
        if (Joint_AngleSet_r[i]-Angle_limitMax_r[i]>=EPSINON)
        {
            Joint_AngleSet_r[i]=Angle_limitMax_r[i];
            cout << "Joint_AngleSet_r " << Joint_AngleSet_r[i]*180.0/Pi << " >= "
                 << "Angle_limitMax_r " << Angle_limitMax_r[i]*180.0/Pi << " i=" << i << endl;
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

    Joint_Angle[0]=Joint_AngleSet_r[0]*DM[0]+Pi/3;
    Joint_Angle[1]=Joint_AngleSet_r[1]*DM[1]+Pi/6;
    Joint_Angle[2]=Joint_AngleSet_r[2]*DM[2]+Pi/2;
    Joint_Angle[3]=Joint_AngleSet_r[3]*DM[3]+Pi/2;
    Joint_Angle[4]=Joint_AngleSet_r[4]*DM[4]+Pi/2;
    Joint_Angle[5]=Joint_AngleSet_r[5]*DM[5]+Pi * 170 /180.0; //   170               //第六个关节340连续旋转
    Joint_Angle[6]=Joint_AngleSet_r[6];      //Joint_AngleSet[6]以0x0c3b为基准，小于这个值张开，大于于闭合

    //double c;
    for(int i=0; i<7; i++)
    {
        cout << "Joint_Angle[i]" << " " << Joint_Angle[i]*180.0/Pi << endl;
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
            //mCmd.joint[i].pos=(ushort) ((sqrt(Delte[i])-L0[i])*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) ;
            ///李玲珑硕士论文P15～17,直线缸的伸长量与传感器电压值成反比
            mCmd.joint[i].pos=(ushort)((Trip[i]-(sqrt(Delte[i])-L0[i]))*(limitMax[i]-limitMin[i])/Trip[i]+limitMin[i]) ;
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

