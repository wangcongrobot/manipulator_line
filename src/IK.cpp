#include "IK.h"
#include "parser.h"

std::vector<double> IK(double q_current[], double x, double y, double z, int step_num)
{
    std::vector<double> joint_values_ik;
    Eigen::VectorXd qPre(6);

    qPre << q_current[0],q_current[1],q_current[2],q_current[3],q_current[4],q_current[5];

    Parser parser;
    Eigen::Matrix4d transformation = parser.Foward(qPre); // current position and orientation
    int number1 = step_num, number2 = 10;
    double distanceX = x;
    double distanceY = y;
    double distanceZ = z;
    for (int i = 0; i < number1; i++)
    {
        transformation(0, 3) += distanceX / number1;
        transformation(1, 3) += distanceY / number1;
        transformation(2, 3) += distanceZ / number1;
        Eigen::VectorXd q(6);
        q = parser.Inverse(transformation, qPre);
        // cout << q << endl;
        if (q(0) > 10) continue;

        for(int j=0; j<number2; j++)
        {
            for(int k=0; k<6; k++)
            {
                joint_values_ik.push_back(qPre(k)+(q(k)-qPre(k))/number2*j);
            }

        }
        //cout << "q\n" << q << endl;
        cout << "q: \n" << q*180/Pi << endl;
        qPre = q;
        //transformation = parser.Foward(qPre);
        //cout << transformation << endl;
        //cout << endl;
    }

    return joint_values_ik;
}

void test(double q_current[])
{
	Parser parser;
	// Jaco physical angles, modify this to calculate other angular position
	Eigen::VectorXd q_sia7f(6);
	q_sia7f << q_current[0],q_current[1],q_current[2],q_current[3],q_current[4],q_current[5];
	cout << "Manipulator' current joint angle is\n" << q_sia7f*180/Pi << "\n\n";

	Eigen::Matrix4d t1 = parser.Foward(q_sia7f);
	cout << "End position and attitude is\n" << t1 << "\n\n";

	//Eigen::MatrixXd Jacob0 = parser.Jacob0(q_sia7f);
	//cout << "Jacobian matrix with respect to base: \n" << Jacob0 << "\n\n";

	//Eigen::MatrixXd Jacobn = parser.Jacobn(q_sia7f);
	//cout << "Jacobian matrix with respect to end: \n" << Jacobn << "\n\n";

	Eigen::VectorXd angle = parser.Inverse(t1, q_sia7f);
	//cout << "Inverse solution of joint angle: \n" << angle << "\n\n";

	//system("pause");

}
