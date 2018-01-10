#include "parser.h"

using namespace std;

Parser::Parser()
{
    // DH Parameters of 7-F manipulator
    double a1 = 0.0925;
    double a2 = 0.950;
    double a3 = 0.127;
    double d4 = 0.522;
    double d6 = 0.450;

    // DH is DH model of the manipulator. Rows are DOF, cols are alpha/a/theta/d.
    // The angle uses radian, the distance uses mm.
    Eigen::MatrixXd dh(6, 4);
    //      alpha    a   theta   d
    dh <<  Pi / 2,   a1,    0,   0,
    0,   a2,    0,   0,
    Pi / 2,   a3,    0,   0,
    -Pi / 2,    0,    0,  d4,
    Pi / 2,    0,    0,   0,
    0,    0,    0,  d6;

    this->dh = dh;
}
/*

DH
q1    Pi / 2, 0,  0, d1,
q2    Pi,     d2, 0, 0,
q3    Pi / 2, 0,  0, -e2,
q4    2 * aa, 0,  0, -d4b,
q5    2 * aa, 0,  0, -d5b,
q6    Pi,     0,  0, -d6b;

*/

Eigen::MatrixXd Parser::Foward(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ = JointAngle2DhAngle(q);
    return DhFoward(dh, dhQ);
}

Eigen::MatrixXd Parser::Jacobn(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ = JointAngle2DhAngle(q);
    return DhJacobn(dh, dhQ);
}

Eigen::MatrixXd Parser::Jacob0(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ = JointAngle2DhAngle(q);
    return DhJacob0(dh, dhQ);
}

Eigen::VectorXd Parser::Inverse(Eigen::MatrixXd t, Eigen::VectorXd qR)
{
    Eigen::VectorXd dhQR = JointAngle2DhAngle(qR);
    return DhInverse(dh, t, dhQR);
}

// Transformation from DH algorithm to Jaco physical angles, p4
Eigen::VectorXd Parser::JointAngle2DhAngle(Eigen::VectorXd q)
{
    Eigen::VectorXd dhQ(6);
    dhQ << q(0), q(1), q(2), q(3), q(4), q(5);

    return dhQ;
}

// Transformation from Jaco physical angles to DH algorithm, p4
Eigen::VectorXd Parser::DhAngle2JointAngle(Eigen::VectorXd dhQ)
{
    Eigen::VectorXd q(6);
    q << dhQ(0), dhQ(1), dhQ(2), dhQ(3), dhQ(4), dhQ(5);

    return q;
}

// ÑØx¡¢y¡¢zÈýžö·œÏòµÄÆœÒÆÁ¿£¬¹¹³ÉÆœÒÆÏòÁ¿
// Ïò4X4ÆëŽÎŸØÕóÌî³äÆœÒÆÏòÁ¿
// ÆëŽÎ±ä»»ŸØÕóµÚËÄÁÐŒŽÆœÒÆÏòÁ¿
Eigen::Matrix4d Parser::Translation(Eigen::Vector3d p)
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity(4, 4); //4x4µ¥Î»ŸØÕó

    t(0, 3) = p(0);
    t(1, 3) = p(1);
    t(2, 3) = p(2);

    return t;
}

// ÑØx¡¢y¡¢zÈýžöÖáÐý×ªµÄÐý×ªŸØÕó
// Ïò4x4ÆëŽÎŸØÕóÌî³äÐý×ªŸØÕó
// ÆëŽÎ±ä»»ŸØÕó×óÉÏ·œ3x3ŸØÕóŒŽÐý×ªŸØÕó
Eigen::Matrix4d Parser::Rotation(char axis, double r) // Ðý×ªÖáaxis£¬Ðý×ªœÇr
{
    Eigen::Matrix4d t = Eigen::Matrix4d::Identity(4, 4); //4x4µ¥Î»ŸØÕó

    double ct = cos(r);
    double st = sin(r);

    switch (axis)
    {
    case 'x':
        t << 1,  0,   0, 0,
        0, ct, -st, 0,
        0, st,  ct, 0,
        0,  0,   0, 1;
        break;
    case 'y':
        t << ct, 0, st, 0,
        0, 1,  0, 0,
        -st, 0, ct, 0,
        0, 0,  0, 1;
        break;
    case 'z':
        t << ct, -st, 0, 0,
        st,  ct, 0, 0,
        0,   0, 1, 0,
        0,   0, 0, 1;
        break;
    }

    return t;
}

// ±ä»»ŸØÕóTÎ¢·ÖÇóµŒ
Eigen::VectorXd Parser::DeltaT(Eigen::MatrixXd t1, Eigen::MatrixXd t2)
{
    Eigen::VectorXd diff(6);
    Eigen::VectorXd cross = Eigen::VectorXd::Zero(3);

    for (int i = 0; i < 3; i++)
    {
        Eigen::Vector3d x1, x2;
        x1 << t1(0, i), t1(1, i), t1(2, i);
        x2 << t2(0, i), t2(1, i), t2(2, i);

        cross += x1.cross(x2); // x1Óëx2œøÐÐ²æ³ËŒÆËã
    }

    for (int i = 0; i < 3; i++)
    {
        diff(i) = t2(i, 3) - t1(i, 3);
        diff(i + 3) = 0.5*cross(i);
    }

    return diff;
}

// žùŸÝDH²ÎÊýÒÔŒ°¹ØœÚ±äÁ¿ÇóÔË¶¯Ñ§Õýœâ
/*    alpha   a theta d
dh << Pi / 2, a1, 0,  0,
		   0, a1, 0,  0,
      Pi / 2, a1, 0,  0,
	 -Pi / 2,  0, 0, d4,
	  Pi / 2,  0, 0,  0,
		   0,  0, 0, d6;
*/
Eigen::MatrixXd Parser::DhFoward(Eigen::MatrixXd dh, Eigen::VectorXd dhQ)
{
    Eigen::Matrix4d t60 = Eigen::Matrix4d::Identity(4, 4); // 4x4µ¥Î»ŸØÕó

    for (int i = 0; i < dh.rows(); i++)
    {
        Eigen::Vector3d px,pz;
        px << dh(i, 1), 0, 0;
        pz << 0, 0, dh(i, 3);
        dh(i, 2) += dhQ(i);
        // cout << dh << endl;
        t60 *= Rotation('z', dh(i, 2)) * Translation(pz) * Translation(px) * Rotation('x', dh(i, 0));
        // cout << "t" << i+1 << t60 << endl;
    }

    return t60; // Ä©¶ËT6Ïà¶ÔÓÚ»ù×ø±êÏµT0µÄ±ä»»ŸØÕó
}
// Çó6x6ÑÅ¿Ë±È
Eigen::MatrixXd Parser::DhJacobn(Eigen::MatrixXd dh, Eigen::VectorXd dhQ)
{
    Eigen::MatrixXd jn = Eigen::MatrixXd::Identity(6, 6); //6x6µ¥Î»ŸØÕó

    Eigen::Matrix4d t = Eigen::Matrix4d::Identity(4, 4);
    for (int i = dh.rows() - 1; i >= 0; i--)
    {
        Eigen::Vector3d pp;
        pp << dh(i, 1), 0, dh(i, 3);
        dh(i, 2) += dhQ(i);
        t = Rotation('z', dh(i, 2)) * Translation(pp) * Rotation('x', dh(i, 0)) * t;

        Eigen::Vector3d a, o, n, p;
        a << t(0, 0), t(1, 0), t(2, 0);
        o << t(0, 1), t(1, 1), t(2, 1);
        n << t(0, 2), t(1, 2), t(2, 2);
        p << t(0, 3), t(1, 3), t(2, 3);

        jn(0, i) = -a(0) * p(1) + a(1) * p(0);// ÆœÒÆ
        jn(1, i) = -o(0) * p(1) + o(1) * p(0);
        jn(2, i) = -n(0) * p(1) + n(1) * p(0);
        jn(3, i) = a(2);// Ðý×ª
        jn(4, i) = o(2);
        jn(5, i) = n(2);
    }

    return jn;
}

Eigen::MatrixXd Parser::DhJacob0(Eigen::MatrixXd dh, Eigen::VectorXd dhQ)
{
    Eigen::MatrixXd j0 = DhJacobn(dh, dhQ);

    Eigen::MatrixXd t0 = DhFoward(dh, dhQ);
    Eigen::MatrixXd t(6, 6);
    t << t0(0, 0), t0(0, 1), t0(0, 2),         0,       0,        0,
    t0(1, 0), t0(1, 1), t0(1, 2),         0,		0,		  0,
    t0(2, 0), t0(2, 1), t0(2, 2),		   0,		0,		  0,
    0,        0,        0,  t0(0, 0), t0(0, 1), t0(0, 2),
    0,        0,        0,  t0(1, 0), t0(1, 1), t0(1, 2),
    0,        0,        0,  t0(2, 0), t0(2, 1), t0(2, 2);
    j0 = t * j0;

    return j0;
}

Eigen::VectorXd Parser::DhInverse(Eigen::MatrixXd dh, Eigen::MatrixXd t, Eigen::VectorXd dhQR)
{
    Eigen::VectorXd qR = DhAngle2JointAngle(dhQR);
    Eigen::VectorXd q(6);
    Eigen::VectorXd dhQ = dhQR;
    Eigen::VectorXd deltaQ;

    double norm = 1.0;
    double min = 1e-12; // Stop condition
    int count = 0;
    int maxCount = 1000; // Max number of iterations

    while (norm > min)
    {
        Eigen::VectorXd e = DeltaT(DhFoward(dh, dhQ), t);
        Eigen::MatrixXd j0 = DhJacob0(dh, dhQ);
        // Only for latest version of Eigen
        // Eigen::VectorXd dDhQ = (j0.pinv()) *e;
        Eigen::VectorXd dDhQ = j0.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(e);

        dhQ += dDhQ;
        for (int i = 0; i < dhQ.rows(); i++)
            dhQ(i) -= ((int)(dhQ(i) / (2 * Pi))) * 2 * Pi;
        norm = dDhQ.norm();

        count++;
        if (count > maxCount)
        {
            q << 10 * Pi, 0, 0, 0, 0, 0;
            return q;
        }
    }

    q = DhAngle2JointAngle(dhQ);
    deltaQ = q - qR;

    for (int i = 0; i < q.rows(); i++)
    {
        deltaQ(i) -= ((int)(deltaQ(i) / (2 * Pi))) * 2 * Pi;
        if (deltaQ(i) < -Pi)
            deltaQ(i) += 2 * Pi;
        else if
        (deltaQ(i) > Pi) deltaQ(i) -= 2 * Pi;
    }

    return qR + deltaQ;
}
