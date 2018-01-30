#ifndef DATA_H_INCLUDED
#define DATA_H_INCLUDED
typedef struct
{
    unsigned char com0Recved:1;
    unsigned char com0Sending:1;
} BIT;
typedef struct
{
    unsigned short pos:12;//控制命令 位置
    unsigned short mode:3;//运动模式
    unsigned short frz:1;//冻结状态
} MSJOINT; //主手给从手的关节信息
typedef struct
{
    unsigned char head;//'M'
    unsigned char id;//用来指示该包数据的含义

    //一下7个量中的2个或者多个，可能会被复用
    MSJOINT joint[7];//底座摆动 大臂俯仰 肘关节俯仰 前臂旋转 腕关节俯仰 腕关节旋转 夹钳夹持
    //MSJOINT Azimuth;//底座摆动
    //MSJOINT Shoulder;//大臂俯仰
    //MSJOINT ELbowPi;//肘关节俯仰
    //MSJOINT ForeArm;//前臂旋转 没有极限位
    //MSJOINT WristPi;//腕关节俯仰
    //MSJOINT WristRo;//腕关节旋转 没有极限位 腕转模式 速度-位置
    //MSJOINT JawClamp;//夹钳夹持 夹钳模式 位置-开关-夹紧-锁紧

    unsigned char Frz:2;//从手激活-冻结-锁定
    unsigned char Hydro:1;//液压开启-关断
    unsigned char DynMod:1;//动态特性 快速-慢速
    unsigned char MoveRatio:4;//运动比例 10%-20%～100%紧

    unsigned char JawSpd:2;//夹钳速度 最慢-慢速-快速-最快
    unsigned char Res1:6;//留待扩展
    //第19个字节 参数设置时使用
    unsigned char key:3;//按下的是那个键
    unsigned char index:5;//当前对应的参数编号，收卷时为0；
    //第20个字节
    unsigned char asterisk;//'*'
    unsigned short checksum;//校验和 前面所有字的异或^
    unsigned short END;//"\r\n"回车换行
} MCMD; //主手发送命令信息

typedef struct
{
    unsigned short pos:12;//关节反馈值 位置
    unsigned short ctrlErr:1;//控制错误
    unsigned short contiErr:1;//采样连续性错误
    unsigned short overErrA:1;//超限错误
    unsigned short overErrB:1;//超限错误
} SMJOINT; //从手给主手的关节信息
typedef struct
{
    unsigned char head;//'S'
    unsigned char id;//用来指示该包数据的含义

    SMJOINT joint[7];//底座摆动 大臂俯仰 肘关节俯仰 前臂旋转 腕关节俯仰 腕关节旋转 夹钳夹持
    //第17字节
    unsigned char timeoutErr:1;//接收主手数据超时
    unsigned char telemErr:7;//和关节失去联系 感觉没必要
    //第18、19字节
    unsigned char res1;//保留
    unsigned char echo;//保留
    //第20字节
    unsigned char asterisk;//'*'
    unsigned short checksum;//校验和 前面所有字的异或^
    unsigned short END;//"\r\n"回车换行
} SFEED; //从手反馈信息 发送完毕大约需要12毫秒

typedef struct
{
    unsigned char head;//'S'
    unsigned char id;//用来指示该包数据的含义

    unsigned short scmdPos[7];//底座摆动 大臂俯仰 肘关节俯仰 前臂旋转 腕关节俯仰 腕关节旋转 夹钳夹持控制量信息
    //第17字节
    unsigned char timeoutErr:1;//接收主手数据超时
    unsigned char telemErr:7;//和关节失去联系 感觉没必要
    //第18、19字节
    unsigned char res1;//保留
    unsigned char res2;//保留
    //第20字节
    unsigned char asterisk;//'*'
    unsigned short checksum;//校验和 前面所有字的异或^
    unsigned short END;//"\r\n"回车换行
} SPOS; //从手位置反馈信息 发送完毕大约需要12毫秒

//零点设置 从手软件所用结构
//定义结构
typedef struct
{
    unsigned char head;//'M' or 'S'
    unsigned char id;//用来指示该包数据的含义 编号从100开始
    unsigned short para[8];//7个零漂 7个最小位置 7个最大位置 16*7个伸卷路径值
    unsigned char index;//保留
    unsigned char asterisk;//'*'
    unsigned short checksum;//校验和 前面所有字的异或^
    unsigned short END;//"\r\n"回车换行
} PARAINFO; //参数设置信息

extern volatile BIT Bit;
extern MCMD mCmd;
extern SFEED sFeed;
extern SPOS  sPos;
extern SFEED  *sfeed;
extern SPOS  *spos;

#endif // DATA_H_INCLUDED
