#include "data.h"
#include "utilities.h"

MCMD mCmd;
SFEED sFeed;
SPOS sPos; //关节角度设定值
SFEED *sfeed;
SPOS  *spos; //关节角度实际值


volatile int TIMER_FLAG=0;
