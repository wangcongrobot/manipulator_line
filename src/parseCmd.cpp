/******************
 * 命令解析
 *
 ******************/

#include "parseCmd.h"
#include "serial.h"
#include "createFile.h"
#include "motionControl.h"
/*
//TODO 此函数一直被调用，spos一直在高速刷新
void parseCmd() //判断接收到一条完整的数据条
{
    unsigned char Checked_OK=0;
    receiveFromSerial(24); //从串口接收一包24字节的数据
    if (Bit.com0Recved==1)
    {
        Checked_OK=CheckRecvData(); //对数据进行校验
        if (Checked_OK==1)
        {
            if(com0RecvBuf[1]==0x01)   // 反馈值是角度设定值
            {
                memcpy(&sPos,com0RecvBuf,sizeof(SPOS));
                // SPOS *spos=(SPOS*)&(com0RecvBuf[0]);//(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
            }
            if(com0RecvBuf[1]==0x00)   // 反馈值是角度实际值，包含各种故障诊断信息
            {
                uart0_receive_ok=1;  // 收到正确格式的24个字节标志位，并且是关节实际值

                spos=(SPOS*)&(com0RecvBuf[0]); // (char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
                //cout << "feedback joint: ";
                for(int i=0; i<7; i++)
                {
                    //cout << spos->scmdPos[i] << " ";
                }
                //cout << endl;
                //dataRecord(); //把接收到的数据存到文件里面
                //printf("Data_Record() successfully!\n");

                //getCurrentJoint();
                //printf("getCurrentJoint() successfully!\n");
            }
            //校验成功后，将反馈数据保存到文件中
            dataRecord();
        }
        else
        {
            Bit.com0Recved=0;    //若校验不成功，再次允许接收数据
            printf("check receive error!\n");
        }
    }

    //printf("ParseCmd successfully!\n");
}
*/


