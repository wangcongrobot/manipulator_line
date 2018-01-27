#include "Test.h"
#include "ConvertAngle.h"
#include "data.h"
#include "parameters.h"

double test_cmd_joint[7] = {0.1, 1.0, -0.5,  0.1, 1.0,-0.5, 0x0c3b};
void Test()
{
    for(int i=0;i<7;i++)
    {
        Joint_AngleSet_r[i]=test_cmd_joint[i];
    }
    AngleConvert();

}
