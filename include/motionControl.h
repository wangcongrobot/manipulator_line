#ifndef MOTIONCONTROL_CREATE_H
#define MOTIONCONTROL_CREATE_H

void sendCtrl();
void Run();
void sInit();
void getCurrentJoint();
int motionControl();
void AngleConvert();     // 关节角度更新函数
void parse();

#endif // MOTIONCONTROL_CREATE_H
