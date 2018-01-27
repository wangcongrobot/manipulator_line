#include "Test.h"
#include "ConvertAngle.h"

unsigned short test_cmd_joint[7] = {100, 200, 300, 400, 500, 600, 700};
void Test()
{
    for(int i=0;i<7;i++)
    {
        Joint_AngleSet_r[i]=test_cmd_joint[i];
    }

}
