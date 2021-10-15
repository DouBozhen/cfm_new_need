#include <algorithm>
#include <math.h>
#include <vector>
#include <iterator>
#include "ur_kinematics.h"
#include "math_calculate.h"

using namespace math_calculate;
using namespace std;

const double d1 = 89.159;
const double a2 = -425;
const double a3 = -392.25;
const double d4 = 109.15;
const double d5 = 94.65;
const double d6 = 82.3;

void UrKinematics::FK(ur_data_type::Joint joint, ur_data_type::CartPose &pose, double R06_33[3][3])
{
    double q[6];
    for (int j = 0; j != 6; ++j)
    {
        q[j] = joint.jVal[j];
    }

    std::array<double, 6> tcp_pose = {};
    urForward(q, tcp_pose, R06_33);

    pose.point.x = tcp_pose[0];
    pose.point.y = tcp_pose[1];
    pose.point.z = tcp_pose[2];   
    pose.rpy.rx = tcp_pose[3];
    pose.rpy.ry = tcp_pose[4];
    pose.rpy.rz = tcp_pose[5];   
}

void UrKinematics::getRotationByJoint(ur_data_type::Joint joint,  double R06_33[3][3])
{
    double q[6];
    for (int j = 0; j != 6; ++j)
    {
        q[j] = joint.jVal[j];
    }

    std::array<double, 6> tcp_pose;


    urForward(q, tcp_pose, R06_33);
}

bool UrKinematics::IK(ur_data_type::CartPose pose, ur_data_type::Joint joint_ref, ur_data_type::Joint &joint_target)
{
    double tcp_pose[6];
    tcp_pose[0] = pose.point.x;
    tcp_pose[1] = pose.point.y;
    tcp_pose[2] = pose.point.z;   
    tcp_pose[3] = pose.rpy.rx;
    tcp_pose[4] = pose.rpy.ry;
    tcp_pose[5] = pose.rpy.rz;  

    double ref[6];
    for (int j = 0; j != 6; ++j)
    {
        ref[j] = joint_ref.jVal[j];
    }

    double target[6] = {};
    double error;
    if (!urInverse(tcp_pose, ref, target, error))
    {
        return false;
    }

    for (int j = 0; j != 6; ++j)
    {
        joint_target.jVal[j] = target[j];
    }
    return true;
}

bool UrKinematics::urInverse(double tcp_pose[6],double joint_ref[6], double joint_target[6], double error_min)
{
    //double tcp_pose[],double itheta[][6]
    //double tcp_pose[6],double joint_ref[6],double itheta[8][6],
    //double joint_target[6];

    //double tcp_pose[6];
    double itheta[8][6];
    //double tcp_pose[6]={-367.11,348.2,166.92,3.132,-0.002,-1.815};
    //double T_goal_m[6]={-367.11,348.2,166.92,3.132,-0.002,-1.815};
    //-423.27,-284.80,101.98,-3.08,-0.004,-0.454
    //double joint_ref[6]={-56.12,-76.49,116.82,-130.83,-89.34,138};
    double P[3];
    double Trot[4][4];
    double R1[3][3];
    double num[4] = { 0,0,0,1 };
    //function itheta=ikine_UR5(tcp_pose)
    // tcp_pose是末端姿态，1×6，[X,Y,Z,A,B,C]
    //double itheta[48]是逆解的角度，8×6,单位是°
    // P 是目标位置，3×1
    // R1 是旋转矩阵3×3
    // Trot 是齐次变换矩阵4×4
    P[0] = tcp_pose[0] * 1000;
    P[1] = tcp_pose[1] * 1000;
    P[2] = tcp_pose[2] * 1000;

    double tcp_pose1[] = { tcp_pose[3],tcp_pose[4],tcp_pose[5] };

    rpy2Rotation(tcp_pose1, R1);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            Trot[i][j] = R1[i][j];
        }
    }

    for (int i = 0; i < 3; i++)
    {
        Trot[i][3] = P[i];
    }
    for (int i = 0; i < 4; i++)
    {

        Trot[3][i] = num[i];
    }

    int theta7 = 0;
    double a6 = 0;
    int afa6 = 0;

    // double T67[16]={cos(theta7),-sin(theta7),0,a6,sin(theta7)*cos(afa6),cos(theta7)*cos(afa6),-sin(afa6),-sin(afa6)*d6,sin(theta7)*sin(afa6),cos(theta7)*sin(afa6),cos(afa6),cos(afa6)*d6,0,0,0,1};
    double T67_44[4][4] = { {cos(theta7),-sin(theta7),0,a6},{sin(theta7)*cos(afa6),cos(theta7)*cos(afa6),-sin(afa6),-sin(afa6)*d6},{sin(theta7)*sin(afa6),cos(theta7)*sin(afa6),cos(afa6),cos(afa6)*d6},{0,0,0,1} };
    //double *T67_inv;

    double T06[4][4];
    //difine a inv_matrix of T67
    double T67_inv44[4][4];
    //double smallest;

    //minimum index
    int index;
    //inverse matrix calculate
    matrix_inverse_44_.invMatrix44ByGaussianElimination(T67_44, T67_inv44);
    // UrMathCalculate::GetMatrixInverse(T67_44, 4, T67_inv44);

    //            T67_inv= LUP_solve_inverse(T67);

    //            for (int r=0;r<4;r++)
    //                {
    //                    for(int q=0;q<4;q++)
    //                    {
    //                        T67_inv44[r][q]=T67_inv[r*4+q];
    //                    }
    //                }

    // two 4*4 matrix multiply
    for (int m = 0; m < 4; m++)
    {
        for (int s = 0; s < 4; s++)
        {
            T06[m][s] = 0;
            for (int n = 0; n < 4; n++)
            {
                T06[m][s] += Trot[m][n] * T67_inv44[n][s];
            }
        }
    }


    //=Trot/T67;
    //%将T06各元素提取出来赋值
    double nx = T06[0][0];
    double ny = T06[1][0];
    double ox = T06[0][1];
    double oy = T06[1][1];
    double ax = T06[0][2];
    double ay = T06[1][2];
    double az = T06[2][2];
    double px = T06[0][3];
    double py = T06[1][3];
    double pz = T06[2][3];
    //%%%%%%初始化itheta矩阵
    //%定义变量k,其代表逆解的行名称
    //%求解theta1
    double ForJudgment = px * px + py * py - d4 * d4;

    if (ForJudgment < -0.000001)
    {
        printf("超出工作空间，无法求解\n");
        return false;
    }
    else
    {
        //%求解theta1
        if (ForJudgment >= -0.000001&&ForJudgment < 0)
            ForJudgment = 0;
        double theta1_1 = atan2(py, px) - atan2(-d4, sqrt(ForJudgment));
        double theta1_2 = atan2(py, px) - atan2(-d4, -sqrt(ForJudgment));
        //%求解theta5
        double S5_1 = sqrt(pow((-sin(theta1_1)*nx + cos(theta1_1)*ny), 2) + pow((-sin(theta1_1)*ox + cos(theta1_1)*oy), 2));
        double theta5_1 = atan2(S5_1, sin(theta1_1)*ax - cos(theta1_1)*ay);
        double S5_2 = -sqrt(pow((-sin(theta1_1)*nx + cos(theta1_1)*ny), 2) + pow((-sin(theta1_1)*ox + cos(theta1_1)*oy), 2));
        double theta5_2 = atan2(S5_2, sin(theta1_1)*ax - cos(theta1_1)*ay);
        double S5_3 = sqrt(pow((-sin(theta1_2)*nx + cos(theta1_2)*ny), 2) + pow((-sin(theta1_2)*ox + cos(theta1_2)*oy), 2));
        double theta5_3 = atan2(S5_3, sin(theta1_2)*ax - cos(theta1_2)*ay);
        double S5_4 = -sqrt(pow((-sin(theta1_2)*nx + cos(theta1_2)*ny), 2) + pow((-sin(theta1_2)*ox + cos(theta1_2)*oy), 2));
        double theta5_4 = atan2(S5_4, sin(theta1_2)*ax - cos(theta1_2)*ay);
        //%当S5不等于0时可求出如下theta6,需要判断语句,并求解theta234的和， %求解theta2，%求解theta23的和,%求解theta4=theta234-theta23,%求解theta3=theta23-theta2,%将解分成8组解，以弧度为单位
        int k = 0;
        double S234[4];
        double C234[4];
        double theta234[4];
        double B1[4];
        double B2[4];
        double A[4];
        double B[4];
        double C[4];
        //double itheta[8][6];
        double theta2[8];
        double theta23[8];
        double theta4[8];
        double theta3[8];
        if (fabs(S5_1) > 0.000001)
        {
            double theta6_1 = atan2((-sin(theta1_1)*ox + cos(theta1_1)*oy) / S5_1, (sin(theta1_1)*nx - cos(theta1_1)*ny) / S5_1);

            S234[0] = -az / S5_1;

            C234[0] = -(cos(theta1_1)*ax + sin(theta1_1)*ay) / S5_1;
            theta234[0] = atan2(S234[0], C234[0]);
            B1[0] = cos(theta1_1)*px + sin(theta1_1)*py - d5 * S234[0];
            B2[0] = pz - d1 + d5 * C234[0];
            A[0] = -2 * B2[0] * a2;
            B[0] = 2 * B1[0] * a2;
            C[0] = pow(B1[0], 2) + pow(B2[0], 2) + pow(a2, 2) - pow(a3, 2);
            if (pow(A[0], 2) + pow(B[0], 2) - pow(C[0], 2) >= 0)
            {
                theta2[0] = atan2(B[0], A[0]) - atan2(C[0], sqrt(pow(A[0], 2) + pow(B[0], 2) - pow(C[0], 2)));
                theta2[1] = atan2(B[0], A[0]) - atan2(C[0], -sqrt(pow(A[0], 2) + pow(B[0], 2) - pow(C[0], 2)));
                theta23[0] = atan2((B2[0] - a2 * sin(theta2[0])) / a3, (B1[0] - a2 * cos(theta2[0])) / a3);
                theta23[1] = atan2((B2[0] - a2 * sin(theta2[1])) / a3, (B1[0] - a2 * cos(theta2[1])) / a3);
                theta4[0] = theta234[0] - theta23[0];
                theta4[1] = theta234[0] - theta23[1];
                theta3[0] = theta23[0] - theta2[0];
                theta3[1] = theta23[1] - theta2[1];
                itheta[k + 0][0] = theta1_1;
                itheta[k + 0][1] = theta2[0];
                itheta[k + 0][2] = theta3[0];
                itheta[k + 0][3] = theta4[0];
                itheta[k + 0][4] = theta5_1;
                itheta[k + 0][5] = theta6_1;

                itheta[k + 1][0] = theta1_1;
                itheta[k + 1][1] = theta2[1];
                itheta[k + 1][2] = theta3[1];
                itheta[k + 1][3] = theta4[1];
                itheta[k + 1][4] = theta5_1;
                itheta[k + 1][5] = theta6_1;

                k = k + 2;
            }
        }
        if (fabs(S5_2) > 0.00001)
        {

            double theta6_2 = atan2((-sin(theta1_1)*ox + cos(theta1_1)*oy) / S5_2, (sin(theta1_1)*nx - cos(theta1_1)*ny) / S5_2);
            //theta6_2=atan2((-sin(theta1_1)*ox+cos(theta1_1)*oy)/S5_2,(sin(theta1_1)*nx-cos(theta1_1)*ny)/S5_2);
            S234[1] = -az / S5_2;
            C234[1] = -(cos(theta1_1)*ax + sin(theta1_1)*ay) / S5_2;
            theta234[1] = atan2(S234[1], C234[1]);

            B1[1] = cos(theta1_1)*px + sin(theta1_1)*py - d5 * S234[1];
            B2[1] = pz - d1 + d5 * C234[1];
            A[1] = -2 * B2[1] * a2;
            B[1] = 2 * B1[1] * a2;

            C[1] = pow(B1[1], 2) + pow(B2[1], 2) + pow(a2, 2) - pow(a3, 2);

            if (pow(A[1], 2) + pow(B[1], 2) - pow(C[1], 2) >= 0)
            {


                theta2[2] = atan2(B[1], A[1]) - atan2(C[1], sqrt(pow(A[1], 2) + pow(B[1], 2) - pow(C[1], 2)));
                theta2[3] = atan2(B[1], A[1]) - atan2(C[1], -sqrt(pow(A[1], 2) + pow(B[1], 2) - pow(C[1], 2)));
                theta23[2] = atan2((B2[1] - a2 * sin(theta2[2])) / a3, (B1[1] - a2 * cos(theta2[2])) / a3);
                theta23[3] = atan2((B2[1] - a2 * sin(theta2[3])) / a3, (B1[1] - a2 * cos(theta2[3])) / a3);

                theta4[2] = theta234[1] - theta23[2];
                theta4[3] = theta234[1] - theta23[3];
                theta3[2] = theta23[2] - theta2[2];
                theta3[3] = theta23[3] - theta2[3];

                itheta[k + 0][0] = theta1_1;
                itheta[k + 0][1] = theta2[2];
                itheta[k + 0][2] = theta3[2];
                itheta[k + 0][3] = theta4[2];
                itheta[k + 0][4] = theta5_2;
                itheta[k + 0][5] = theta6_2;

                itheta[k + 1][0] = theta1_1;
                itheta[k + 1][1] = theta2[3];
                itheta[k + 1][2] = theta3[3];
                itheta[k + 1][3] = theta4[3];
                itheta[k + 1][4] = theta5_2;
                itheta[k + 1][5] = theta6_2;
                k = k + 2;


            }
        }
        if (fabs(S5_3) > 0.0001)
        {



            double theta6_3 = atan2((-sin(theta1_2)*ox + cos(theta1_2)*oy) / S5_3, (sin(theta1_2)*nx - cos(theta1_2)*ny) / S5_3);
            S234[2] = -az / S5_3;
            C234[2] = -(cos(theta1_2)*ax + sin(theta1_2)*ay) / S5_3;
            theta234[2] = atan2(S234[2], C234[2]);
            B1[2] = cos(theta1_2)*px + sin(theta1_2)*py - d5 * S234[2];
            B2[2] = pz - d1 + d5 * C234[2];
            A[2] = -2 * B2[2] * a2;
            B[2] = 2 * B1[2] * a2;
            C[2] = pow(B1[2], 2) + pow(B2[2], 2) + pow(a2, 2) - pow(a3, 2);



            if (pow(A[2], 2) + pow(B[2], 2) - pow(C[2], 2) >= 0)
            {


                theta2[4] = atan2(B[2], A[2]) - atan2(C[2], sqrt(pow(A[2], 2) + pow(B[2], 2) - pow(C[2], 2)));
                theta2[5] = atan2(B[2], A[2]) - atan2(C[2], -sqrt(pow(A[2], 2) + pow(B[2], 2) - pow(C[2], 2)));
                theta23[4] = atan2((B2[2] - a2 * sin(theta2[4])) / a3, (B1[2] - a2 * cos(theta2[4])) / a3);
                theta23[5] = atan2((B2[2] - a2 * sin(theta2[5])) / a3, (B1[2] - a2 * cos(theta2[5])) / a3);

                theta4[4] = theta234[2] - theta23[4];
                theta4[5] = theta234[2] - theta23[5];
                theta3[4] = theta23[4] - theta2[4];
                theta3[5] = theta23[5] - theta2[5];

                itheta[k + 0][0] = theta1_2;
                itheta[k + 0][1] = theta2[4];
                itheta[k + 0][2] = theta3[4];
                itheta[k + 0][3] = theta4[4];
                itheta[k + 0][4] = theta5_3;
                itheta[k + 0][5] = theta6_3;

                itheta[k + 1][0] = theta1_2;
                itheta[k + 1][1] = theta2[5];
                itheta[k + 1][2] = theta3[5];
                itheta[k + 1][3] = theta4[5];
                itheta[k + 1][4] = theta5_3;
                itheta[k + 1][5] = theta6_3;
                k = k + 2;
            }
        }
        if (abs(S5_4) > 0.00001)
        {

            double theta6_4 = atan2((-sin(theta1_2)*ox + cos(theta1_2)*oy) / S5_4, (sin(theta1_2)*nx - cos(theta1_2)*ny) / S5_4);
            S234[3] = -az / S5_4;
            C234[3] = -(cos(theta1_2)*ax + sin(theta1_2)*ay) / S5_4;
            theta234[3] = atan2(S234[3], C234[3]);
            B1[3] = cos(theta1_2)*px + sin(theta1_2)*py - d5 * S234[3];
            B2[3] = pz - d1 + d5 * C234[3];
            A[3] = -2 * B2[3] * a2;
            B[3] = 2 * B1[3] * a2;
            C[3] = pow(B1[3], 2) + pow(B2[3], 2) + pow(a2, 2) - pow(a3, 2);


            if (pow(A[3], 2) + pow(B[3], 2) - pow(C[3], 2) >= 0)
            {

                theta2[6] = atan2(B[3], A[3]) - atan2(C[3], sqrt(pow(A[3], 2) + pow(B[3], 2) - pow(C[3], 2)));
                theta2[7] = atan2(B[3], A[3]) - atan2(C[3], -sqrt(pow(A[3], 2) + pow(B[3], 2) - pow(C[3], 2)));
                theta23[6] = atan2((B2[3] - a2 * sin(theta2[6])) / a3, (B1[3] - a2 * cos(theta2[6])) / a3);
                theta23[7] = atan2((B2[3] - a2 * sin(theta2[7])) / a3, (B1[3] - a2 * cos(theta2[7])) / a3);

                theta4[6] = theta234[3] - theta23[6];
                theta4[7] = theta234[3] - theta23[7];
                theta3[6] = theta23[6] - theta2[6];
                theta3[7] = theta23[7] - theta2[7];

                itheta[k + 0][0] = theta1_2;
                itheta[k + 0][1] = theta2[6];
                itheta[k + 0][2] = theta3[6];
                itheta[k + 0][3] = theta4[6];
                itheta[k + 0][4] = theta5_4;
                itheta[k + 0][5] = theta6_4;

                itheta[k + 1][0] = theta1_2;
                itheta[k + 1][1] = theta2[7];
                itheta[k + 1][2] = theta3[7];
                itheta[k + 1][3] = theta4[7];
                itheta[k + 1][4] = theta5_4;
                itheta[k + 1][5] = theta6_4;

                k = k + 2;
            }
        }

        //%将逆解变成角度单位
        if (k > 0)
        {
            for (int i = 0; i < 8; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    itheta[i][j] = itheta[i][j] * 180 / M_PI;
                }
            }

            for (int i = 0; i < k; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    if (itheta[i][j] <= -180)
                    {
                        itheta[i][j] = itheta[i][j] + 360;
                    }
                    else if (itheta[i][j] > 180)
                    {
                        itheta[i][j] = itheta[i][j] - 360;
                    }
                }

            }
        }
        else
        {
            printf("该位姿处于奇异位置有无穷解\n");
            return false;
        }

        //cout<<itheta[0][0]<<" "<<itheta[0][1]<<" "<<itheta[0][2]<<" "<<itheta[0][3]<<" "<<itheta[0][4]<<" "<<itheta[0][5]<<" "<<endl;

        double joint_error[8] = { 0 };

        //compare rad2degree
        for (int i = 0; i < 6; i++)
        {
            joint_ref[i] = joint_ref[i] * 180 / M_PI;
        }



        for (int i = 0; i < 8; i++)
        {

            for (int j = 0; j < 6; j++)
            {
                if (fabs(itheta[i][j] - joint_ref[j]) > 340 && fabs(itheta[i][j] - joint_ref[j]) < 380)
                {
                    joint_error[i] += fabs(fabs(itheta[i][j] - joint_ref[j]) - 360);
                }
                else
                {
                    joint_error[i] += fabs(itheta[i][j] - joint_ref[j]);
                }
            }
        }


        for (int i = 0; i < 6; i++)
        {
            joint_ref[i] = joint_ref[i] * M_PI / 180;
        }

        printf("joint_error: %lf, %lf, %lf, %lf, %lf, %lf\n",
            joint_error[0],
            joint_error[1],
            joint_error[2],
            joint_error[3],
            joint_error[4],
            joint_error[5]);

        auto smallest = std::min_element(begin(joint_error), end(joint_error));

        index = std::distance(joint_error, smallest);
        error_min = *smallest;
        printf("The index is: %d, The min value is: %lf\n", index, error_min);

        for (int i = 0; i < 6; i++)
        {
            joint_target[i] = itheta[index][i] * M_PI / 180;
        }
    }
    return true;
}

void UrKinematics::urForward(const double q[], std::array<double, 6>& tcp, double R06_33[3][3])
{
    double T06[16];
    //double R06[9];
    //q unit rad
    //tcp (X,Y,Z,rx,ry,rz) rpy  unit: m and rad
    double T[16];
    double s1 = sin(*q), c1 = cos(*q); q++;
    double q234 = *q, s2 = sin(*q), c2 = cos(*q); q++;
    double s3 = sin(*q), c3 = cos(*q); q234 += *q; q++;
    q234 += *q; q++;
    double s5 = sin(*q), c5 = cos(*q); q++;
    double s6 = sin(*q), c6 = cos(*q);
    double s234 = sin(q234), c234 = cos(q234);
    T[0] = ((c1*c234 - s1 * s234)*s5) / 2.0 - c5 * s1 + ((c1*c234 + s1 * s234)*s5) / 2.0;
    T[1] = (c6*(s1*s5 + ((c1*c234 - s1 * s234)*c5) / 2.0 + ((c1*c234 + s1 * s234)*c5) / 2.0) -
        (s6*((s1*c234 + c1 * s234) - (s1*c234 - c1 * s234))) / 2.0);
    T[2] = (-(c6*((s1*c234 + c1 * s234) - (s1*c234 - c1 * s234))) / 2.0 -
        s6 * (s1*s5 + ((c1*c234 - s1 * s234)*c5) / 2.0 + ((c1*c234 + s1 * s234)*c5) / 2.0));
    T[3] = ((d5*(s1*c234 - c1 * s234)) / 2.0 - (d5*(s1*c234 + c1 * s234)) / 2.0 -
        d4 * s1 + (d6*(c1*c234 - s1 * s234)*s5) / 2.0 + (d6*(c1*c234 + s1 * s234)*s5) / 2.0 -
        a2 * c1*c2 - d6 * c5*s1 - a3 * c1*c2*c3 + a3 * c1*s2*s3);
    T[4] = c1 * c5 + ((s1*c234 + c1 * s234)*s5) / 2.0 + ((s1*c234 - c1 * s234)*s5) / 2.0;
    T[5] = (c6*(((s1*c234 + c1 * s234)*c5) / 2.0 - c1 * s5 + ((s1*c234 - c1 * s234)*c5) / 2.0) +
        s6 * ((c1*c234 - s1 * s234) / 2.0 - (c1*c234 + s1 * s234) / 2.0));
    T[6] = (c6*((c1*c234 - s1 * s234) / 2.0 - (c1*c234 + s1 * s234) / 2.0) -
        s6 * (((s1*c234 + c1 * s234)*c5) / 2.0 - c1 * s5 + ((s1*c234 - c1 * s234)*c5) / 2.0));
    T[7] = ((d5*(c1*c234 - s1 * s234)) / 2.0 - (d5*(c1*c234 + s1 * s234)) / 2.0 + d4 * c1 +
        (d6*(s1*c234 + c1 * s234)*s5) / 2.0 + (d6*(s1*c234 - c1 * s234)*s5) / 2.0 + d6 * c1*c5 -
        a2 * c2*s1 - a3 * c2*c3*s1 + a3 * s1*s2*s3);
    T[8] = ((c234*c5 - s234 * s5) / 2.0 - (c234*c5 + s234 * s5) / 2.0);
    T[9] = ((s234*c6 - c234 * s6) / 2.0 - (s234*c6 + c234 * s6) / 2.0 - s234 * c5*c6);
    T[10] = (s234*c5*s6 - (c234*c6 + s234 * s6) / 2.0 - (c234*c6 - s234 * s6) / 2.0);
    T[11] = (d1 + (d6*(c234*c5 - s234 * s5)) / 2.0 + a3 * (s2*c3 + c2 * s3) + a2 * s2 -
        (d6*(c234*c5 + s234 * s5)) / 2.0 - d5 * c234);
    T[12] = 0.0; T[13] = 0.0; T[14] = 0.0; T[15] = 1.0;

    double T1[16];
    T1[0] = -1;	T1[1] = 0;   T1[2] = 0;	 T1[3] = 0;
    T1[4] = 0;	T1[5] = -1;  T1[6] = 0;	 T1[7] = 0;
    T1[8] = 0;	T1[9] = 0;   T1[10] = 1; T1[11] = 0;
    T1[12] = 0;	T1[13] = 0;  T1[14] = 0; T1[15] = 1;

    double T2[16];
    T2[0] = 0;	T2[1] = 0;  T2[2] = 1;	T2[3] = 0;
    T2[4] = -1;	T2[5] = 0;  T2[6] = 0;	T2[7] = 0;
    T2[8] = 0;	T2[9] = -1; T2[10] = 0; T2[11] = 0;
    T2[12] = 0;	T2[13] = 0; T2[14] = 0; T2[15] = 1;

    double tmp[16];
    //matrix multily
    matrixMul16(T1, T, tmp);
    matrixMul16(tmp, T2, T06);

    double rpy[3];
    //double T06_33[3][3];

    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            R06_33[i][j] = T06[i * 4 + j];
        }
    }

    rotation2Rpy(R06_33, rpy);

    tcp[0] = T06[3] / 1000;
    tcp[1] = T06[7] / 1000;
    tcp[2] = T06[11] / 1000;
    tcp[3] = rpy[0];
    tcp[4] = rpy[1];
    tcp[5] = rpy[2];
    //return 0;
}

void UrKinematics::mutationJudge(ur_data_type::Joint joint_start, ur_data_type::Joint& joint_mirror)
{
    for (int8_t k = 0; k < 6; k++)
    {
        if (fabs(joint_mirror.jVal[k] - joint_start.jVal[k]) > 1)
        {
            if (joint_mirror.jVal[k] > 0)
            {
                joint_mirror.jVal[k] -= 2 * M_PI;
            }
            else
            {
                joint_mirror.jVal[k] += 2 * M_PI;
            }
        }

        if (fabs(joint_mirror.jVal[k] - joint_start.jVal[k]) > 0.3)
        {
            printf("%d Singular\n", k);
            for (int8_t i = 0; i < 6; i++)
            {
                joint_mirror.jVal[i] = joint_start.jVal[i];
            }

            return;
        }

        if (fabs(joint_mirror.jVal[k] > 1.99 * M_PI))
        {
            printf("Exceeding the range of joint\n");
            for (int8_t i = 0; i < 6; i++)
            {
                joint_mirror.jVal[i] = joint_start.jVal[i];
            }
            return;
        }
    }
}

void UrKinematics::calibrateWorkpiece(double transition_wp2base[4][4])
{
    ur_data_type::CartPose tcp2base;
    tcp2base.point.x = -0.28882;
    tcp2base.point.y = -0.56324;
    tcp2base.point.z = 0.34541;
    tcp2base.rpy.rx = 1.8705;  
    tcp2base.rpy.ry = -1.6991;  
    tcp2base.rpy.rz = -0.4079; 

    ur_data_type::CartPose tcp2workpiece;
    tcp2workpiece.point.x = 0.01618;
    tcp2workpiece.point.y = 0.28168;
    tcp2workpiece.point.z = -0.11777;
    tcp2workpiece.rpy.rx = 5.3075;  
    tcp2workpiece.rpy.ry = -1.5880;  
    tcp2workpiece.rpy.rz = 0.9254;  

    double transition_tcp2base[4][4];
    double transition_tcp2wp[4][4];
    double transition_wp2tcp[4][4];
    getTransitionTcp2Base(tcp2base, transition_tcp2base);
    getTransitionTcp2Base(tcp2workpiece, transition_tcp2wp);
    matrix_inverse_44_.invMatrix44ByGaussianElimination(transition_tcp2wp, transition_wp2tcp);
    matrixMul44(transition_tcp2base, transition_wp2tcp, transition_wp2base);
}