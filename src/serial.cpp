//#include "header.h"
#include "serial.h"

unsigned char uart0_receive_ok=0;
unsigned char len_receive=0;
int nread=0;
int fd;
volatile unsigned char SendEN=0;//发送使能
volatile BIT Bit;

int speed_arr[] = {B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300,
                   B115200, B38400, B19200, B9600, B4800, B2400, B1200, B300
                  };
int name_arr[] = {115200, 38400, 19200, 9600, 4800, 2400, 1200,  300,
                  115200, 38400, 19200, 9600, 4800, 2400, 1200,  300
                 };

////////////////////////////////////////////////////////////////////////////////
/**
*@brief  设置串口通信速率
*@param  fd     类型 int  打开串口的文件句柄
*@param  speed  类型 int  串口速度
*@return  void
*//////////////////////////////////////////////////////////////////////////////

void setSpeed(int fd, int speed)
{
    uint   i;
    int   status;
    struct termios   Opt;
    tcgetattr(fd, &Opt);   //说明：tcgetattr函数用于获取与终端相关的参数。参数fd为终端的文件描述符，返回的结果保存在termios结构体中，该结构体一般
    //包括如下的成员：tcflag_t c_iflag;
    //tcflag_t c_oflag;
    //tcflag_t c_cflag;
    //tcflag_t c_lflag;
    //cc_t     c_cc[NCCS];
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            tcflush(fd, TCIOFLUSH);            //清空终端未完成的输入/输出请求及数据：TCOFLUSH 清除正写入的数据，且不会发送至终端。
            cfsetispeed(&Opt, speed_arr[i]);
            cfsetospeed(&Opt, speed_arr[i]);
            status = tcsetattr(fd, TCSANOW, &Opt);   //TCSANOW：不等数据传输完毕就立即改变属性。
            if  (status != 0)
            {
                perror("tcsetattr fd1"); //用来将上一个函数发生错误的原因输出到标准设备(stderr)参数 s 所指的字符串会先打印出，后面再加上错误原因字符串
                return;
            }
            tcflush(fd,TCIOFLUSH);
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
/**
*@brief   设置串口数据位，停止位和效验位
*@param  fd     类型  int  打开的串口文件句柄
*@param  databits 类型  int 数据位   取值 为 7 或者8
*@param  stopbits 类型  int 停止位   取值为 1 或者2
*@param  parity  类型  int  效验类型 取值为N,E,O,,S
*//////////////////////////////////////////////////////////////////////////////
int setParity(int fd,int databits,int stopbits,int parity)
{
    struct termios options;
    if  ( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return(FALSE);
    }
    options.c_cflag |= (CLOCAL | CREAD);        //一般必设置的标志 CLOCAL:忽略调制解调器线路状态,CREAD:使用接收器
    options.c_cflag &= ~CSIZE;   //c.cflag中cssize代表字符长度
    switch (databits) /*设置数据位数*/
    {
    case 7:
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        fprintf(stderr,"Unsupported data size\n");
        return (FALSE);
    }
    switch (parity)
    {
    case 'n':
    case 'N':
        options.c_cflag &= ~PARENB;   /* Clear parity enable */
        options.c_iflag &= ~INPCK;     /* Enable parity checking */
        break;
    case 'o':
    case 'O':
        options.c_cflag |= (PARODD | PARENB); /* 设置为奇效验*/

        options.c_iflag |= INPCK;             /* Disnable parity checking */
        break;
    case 'e':
    case 'E':
        options.c_cflag |= PARENB;     /* Enable parity */
        options.c_cflag &= ~PARODD;   /* 转换为偶效验*/
        options.c_iflag |= INPCK;       /* Disnable parity checking */
        break;
    case 'S':
    case 's':  /*as no parity*/
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported parity\n");
        return (FALSE);
    }
    /* 设置停止位*/
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB;   //cstopb用来设置两个停止位
        break;
    case 2:
        options.c_cflag |= CSTOPB;
        break;
    default:
        fprintf(stderr,"Unsupported stop bits\n");
        return (FALSE);
    }
    options.c_iflag &= ~(BRKINT | ICRNL | ISTRIP | IXON);  //***关掉流控，不然 0x0d,0x11和0x13会被当做流控特殊字符丢掉**/
    /* Set input parity option */
    if (parity != 'n')
        options.c_iflag |= INPCK;	 // INPACK：允许输入奇偶校验
    tcflush(fd,TCIFLUSH);       	//TCIFLUSH： 清除正收到的数据，且不会读取出来
    options.c_cc[VTIME] = 1; /* 设置超时15 seconds*/     //C_cc控制字符中VTIME：支持的规范模式读取时的超时时间,单位：百毫秒
    options.c_cc[VMIN] = 0; /* Update the options and do it NOW */  //非规范模式读取时的最小字符数

    options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input,选择原始输入模式*/
    options.c_oflag  &= ~OPOST;   /*Output*/

    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("SetupSerial 3");
        return (FALSE);
    }
    return (TRUE);
}

void initSerial()
{
    Bit.com0Recved=0;
    Bit.com0Sending=0;
    fd = open(UART_DEVICE, O_RDWR | O_NOCTTY | O_NDELAY);  //第一个参数时路径。第二个O_RDWR表示可读可写、 O_NOCTTY 选项防止程序受键盘控制中止操作键等影响. O_NDELAY 告诉 UNIX 不必另一端端口是否启用
    if (fd < 0)
    {
        perror(UART_DEVICE);
        exit(1);
    }
    else
    {
        printf("open ");
        printf("%s",ttyname(fd));
        printf(" succesfully\n");
    }
    setSpeed(fd,BAUDRATE);
    setParity(fd,8,1,'N');
    if (setParity(fd,8,1,'N') == FALSE)
    {
        printf("Set Parity Error\n");
        exit (0);  	//exit（0）：正常运行程序并退出程序；exit（1）：非正常运行导致退出程序；
    }
//  fcntl(fd, F_SETFL, FNDELAY);    //串口设置这个选项后,read 调用都是立即返回.没有数据可读时,read 返回 0

}

void writeToSerial(unsigned char *buf,unsigned char len) //把校验位加进去，使用指针强制类型转换方式
{
    //添加校验
    unsigned short *pt=(ushort*)buf;
    unsigned short checksum=pt[0];
    for(unsigned char i=1; i<10; i++)
        checksum^=pt[i];
    pt[10]=checksum;
    write(fd,buf,len);
    Bit.com0Sending=0 ;   //发送完毕 标志位清零

    unsigned char *p;
    printf("\n");
    for(p=buf; p<(buf+24); p++)
    {
        printf("%02x ",*p);
    }
    printf("\nwriteToSerial successfully!\n");
}

void receiveFromSerial( unsigned char length) //接收数据函数
{
    nread=0;
    unsigned char buff_receive[1024];
    unsigned char Data_temp[1024];
    while((nread = read(fd,buff_receive,length))>0 )  //每次接受数据不一定全部接收，有可能由于发的慢，ubuntu接受数据较快，只接收了一部分数据，需要循环读取
    {
        //       printf("nread = %d\n", nread);
        for(unsigned char i = len_receive; i < (len_receive + nread); i++)
        {
            Data_temp[i] = buff_receive[i-len_receive];
        }
        len_receive+=nread;
        if( len_receive>=1024)
        {
            printf("Data received overfow !\n");
            len_receive=0;
            continue;
        }
    }
    for (int i =0; i<len_receive; i++)   //从接受缓冲数组里面截取有用的数据
    {

        if((Data_temp[i]=='S')&&(Data_temp[i+23]=='\n')&&(Bit.com0Recved==0)&&(len_receive>=length))   //24个字节解析完成之后再重新存数据
        {
            memcpy(com0RecvBuf,&Data_temp[i],24);
            Bit.com0Recved=1;       //接收到完整24个数据完成标志位
            len_receive=0;               //重新从头存数据
            printf("Receive 24 bytes OK!\n");
        }
        unsigned char *p;
        //printf("\n");
        for(p=com0RecvBuf; p<(com0RecvBuf+24); p++)
        {
            //printf("%02x ",*p);
        }
        //printf("\n");
    }

    //printf("receiveSerial successfully!\n");
}
unsigned char CheckRecvData()           //校验接收数据函数
{
    unsigned char check_h=0;    //校验和高位
    unsigned char check_l=0;    //校验和低位
    for (unsigned char i=0; i<20; i=i+2)check_l^=com0RecvBuf[i]; //校验和低位
    for (unsigned char i=1; i<20; i=i+2)check_h^=com0RecvBuf[i]; //校验和高位
    if((com0RecvBuf[22]=='\r')&&(com0RecvBuf[21]==check_h)&&(com0RecvBuf[20]==check_l))
    {
        //        printf("check receive OK!\n");
        return 1;
    }
    else
        //      printf("check receive error!\n");
        return 0;
    printf("CheckRecvData successfully!\n");
}
