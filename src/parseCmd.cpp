#include "parseCmd.h"
#include "serial.h"
#include "createFile.h"
#include "motionControl.h"



void parseCmd1( )        //判断接收到一条完整的数据条
{
    unsigned char Checked_OK=0;
    receiveFromSerial(24);
    if (Bit.com0Recved==1)
    {
        Checked_OK=CheckRecvData();
        if (Checked_OK==1)
        {
            if(com0RecvBuf[1]==0x01)   //反馈值是角度设定值
            {
                memcpy(&sPos,com0RecvBuf,sizeof(SPOS));
                cout << "sPos.scmdPos[i]: ";
                for(int i=0; i<7; i++)
                {
                    cout << hex << sPos.scmdPos[i] << " ";
                }
                cout << endl;
                //      SPOS *spos=(SPOS*)&(com0RecvBuf[0]);//(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
            }

            if(com0RecvBuf[1]==0x00)   //反馈值是角度实际值，包含各种故障诊断信息
            {
                uart0_receive_ok=1;       //收到正确格式的24个字节标志位
                spos=(SPOS*)&(com0RecvBuf[0]); //(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
                cout << "spos->scmdPos[i]: ";
                for(int i=0; i<7; i++)
                {
                    cout << hex << spos->scmdPos[i] << " ";
                }
                cout << endl;
                //dataRecord(); //把接收到的数据存到文件里面
                //printf("Data_Record() successfully!\n");

                //getCurrentJoint();
                //printf("getCurrentJoint() successfully!\n");
            }
        }
        else
        {
            Bit.com0Recved=0;    //若校验不成功，再次允许接收数据
            printf("check receive error!\n");
        }
    }

    //printf("ParseCmd successfully!\n");
}

void parse()
{
    while (uart0_receive_ok==0) // waiting for uart receive ok, if ok(==1), jump out
    {
        printf("parse-------------\n");
        sleep(1);
        continue;
    }

    if(com0RecvBuf[1]==0x01)   //反馈值是角度设定值
    {
        memcpy(&sPos,com0RecvBuf,sizeof(SPOS));
        //      SPOS *spos=(SPOS*)&(com0RecvBuf[0]);//(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
    }
    if(com0RecvBuf[1]==0x00)   //反馈值是角度实际值，包含各种故障诊断信息
    {
        spos=(SPOS*)&(com0RecvBuf[0]);//(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
        dataRecord();                      //把接收到的数据存到文件里面
        getCurrentJoint();
        //printf("getCurrentJoint() successfully!\n");

        //********************************
        unsigned char *p;
        for(p=com0RecvBuf; p<(com0RecvBuf+24); p++)
        {
            printf("%x ",*p);
        }
        cout << endl;
    }
    uart0_receive_ok=0;
    Bit.com0Recved=0; //若校验成功，需要解析完之后允许串口接收允许再次接收数据

}

void parseCmd( )        //判断接收到一条完整的数据条
{
    unsigned char Checked_OK=0;
    receiveFromSerial(24);
    if (Bit.com0Recved==1)
    {
        Checked_OK=CheckRecvData();
        if (Checked_OK==1)
        {
            uart0_receive_ok=1;       //收到正确格式的24个字节标志位

            if(com0RecvBuf[1]==0x01)   //反馈值是角度设定值
            {
                memcpy(&sPos,com0RecvBuf,sizeof(SPOS));
                // SPOS *spos=(SPOS*)&(com0RecvBuf[0]);//(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
            }
            if(com0RecvBuf[1]==0x00)   //反馈值是角度实际值，包含各种故障诊断信息
            {
                spos=(SPOS*)&(com0RecvBuf[0]);//(char *)&a:含义就是先取a的首地址,然后强制转换为char指针类型,最后的意思是把数组a转换成char型
            }
        }
        else
        {
            Bit.com0Recved=0;    //若校验不成功，再次允许接收数据
            printf("check receive error!\n");
        }
    }
}
