#include <stdio.h>
#include <cstring>
#include "solve_jacobian.h"

using namespace ur_data_type;
using namespace math_calculate;

bool SloveJacobian::inverseJacobian(
    ur_data_type::Joint position,
    force_data_type::FTSensorData &vw, ur_data_type::Joint &speed
)
{
	double rotation[3][3] = {};
    CartPose actual_tcp;
    ur_kine_.FK(position, actual_tcp, rotation);

    double vw_double_list[6];
    vw_double_list[0] = vw.force.fx;
    vw_double_list[1] = vw.force.fy;
    vw_double_list[2] = vw.force.fz;
    vw_double_list[3] = vw.torque.tx;
    vw_double_list[4] = vw.torque.ty;
    vw_double_list[5] = vw.torque.tz;

    double vw_in_base[6] = {};
    vectorTransBase(vw_double_list, rotation, 6, vw_in_base);
    memcpy(vw_double_list, vw_in_base, sizeof(vw_in_base));

    double jecobian_mat[64] = {};
    jacobi(position.jVal, jecobian_mat);

	double tem_group[6][6] = {};
	auto jecobian_pointer = jecobian_mat;
	for (int i = 0; i < 6*6; i++)
	{
		tem_group[i / 6][i % 6] = *(jecobian_pointer++);
	}

	bool result = ALU(tem_group, vw_double_list, speed.jVal);

    vw.force.fx = vw_double_list[0];
    vw.force.fy = vw_double_list[1];
    vw.force.fz = vw_double_list[2];
    vw.torque.tx = vw_double_list[3];
    vw.torque.ty = vw_double_list[4];
    vw.torque.tz = vw_double_list[5];

    return result;
}

bool SloveJacobian::ALU(double a[6][6], double b[6], double res[6])
{
    double l[6][6] = {};
	double u[6][6] = {};
	int i = 0, r = 0, k = 0;

	//进行U的第一行的赋值
	for (i = 0; i < 6; i++)
	{
		u[0][i] = a[0][i];
	}
	if (abs(u[0][0]) < 1e-5)
	{
		printf("the matrix is unsolvable");
		return false;
	}

	//进行L的第一列的赋值
	for (i = 1; i < 6; i++)
	{
		l[i][0] = a[i][0] / u[0][0];
	}

	//计算U的剩下的行数和L的剩下的列数
	for (r = 1; r < 6; r++)
	{
		for (i = r; i < 6; i++)
		{
			double sum1 = 0;
			for (k = 0; k < r; k++)
			{
				sum1 += l[r][k] * u[k][i];
			}
			u[r][i] = a[r][i] - sum1;
		}

		if (r != 6)
			for (i = r + 1; i < 6; i++)
			{
				double sum2 = 0;
				for (k = 0; k < r; k++)
				{
					sum2 += l[i][k] * u[k][r];
				}
				l[i][r] = (a[i][r] - sum2) / u[r][r];
        }
    }

	double y[6] = {};
	y[0] = b[0];
	for (i = 1; i < 6; i++)
	{
		double sum3 = 0;
		for (k = 0; k < i; k++)
			sum3 += l[i][k] * y[k];
		y[i] = b[i] - sum3;
	}

	res[6 - 1] = y[6 - 1] / u[6 - 1][6 - 1];
	for (i = 6 - 2; i >= 0; i--)
	{
		double sum4 = 0;
		for (k = i + 1; k < 6; k++)
			sum4 += u[i][k] * res[k];
		res[i] = (y[i] - sum4) / u[i][i];
	}

    return true;
}

void SloveJacobian::jacobi(const double * v, double *J)
{
    double a2, a3, d4, d5, d6, v1, v2, v3, v4, v5, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15, t16, t17, t18, t19, t20;
    double t21, t22, t23, t24, t25, t26, t27, t28, t29, t30, t31, t32, t33, t34, t35, t36, t37, t38, t39, t40, t41;
    double t42, t43, t44, t45, t46, t47, t48, t49, t50, t51;
    double *p;

    a2 = par_a_[1];
    a3 = par_a_[2];
    d4 = par_d_[3];
    d5 = par_d_[4];
    d6 = par_d_[5];
    v1 = v[0];
    v2 = v[1];
    v3 = v[2];
    v4 = v[3];
    v5 = v[4];
    t2 = sin(v1);
    t3 = cos(v2);
    t4 = sin(v3);
    t5 = cos(v3);
    t6 = sin(v2);
    t7 = cos(v4);
    t8 = t2 * t3*t4;
    t9 = t2 * t5*t6;
    t10 = t8 + t9;
    t11 = sin(v4);
    t12 = t2 * t3*t5;
    t48 = t2 * t4*t6;
    t13 = t12 - t48;
    t14 = cos(v1);
    t15 = cos(v5);
    t16 = sin(v5);
    t17 = t3 * t4*t14;
    t18 = t5 * t6*t14;
    t19 = t17 + t18;
    t20 = t3 * t5*t14;
    t21 = v2 + v3 + v4;
    t22 = v2 + v3;
    t23 = sin(t22);
    t24 = a3 * t23;
    t25 = a2 * t6;
    t26 = cos(t21);
    t27 = sin(t21);
    t29 = d5 * t26;
    t30 = d6 * t16*t27;
    t28 = t24 + t25 - t29 - t30;
    t31 = -t24 + t29 + t30;
    t32 = v2 + v3 + v4 + v5;
    t33 = sin(t32);
    t34 = cos(t22);
    t35 = a3 * t34;
    t36 = v2 + v3 + v4 - v5;
    t37 = sin(t36);
    t38 = d6 * t37*0.5;
    t39 = d5 * t27;
    t40 = t29 + t30;
    t41 = t2 * t15;
    t42 = t20 - t4 * t6*t14;
    t43 = t11 * t19;
    t44 = t43 - t7 * t42;
    t45 = t16 * t44;
    t46 = t41 + t45;
    t47 = t14 * t15;
    t49 = t7 * t13;
    t50 = t49 - t10 * t11;
    t51 = t16 * t50;
    p = J;
    *p++ = d6 * (t47 + t51) + d4 * t14 - d5 * (t7*t10 + t11 * t13) - a2 * t2*t3 - a3 * t2*t3*t5 + a3 * t2*t4*t6;
    *p++ = -t14 * t28;
    *p++ = t14 * t31;
    *p++ = t14 * t40;
    *p++ = -d6 * t2*t16 - d6 * t3*t5*t7*t14*t15 + d6 * t4*t6*t7*t14*t15 + d6 * t3*t4*t11*t14*t15 + d6 * t5*t6*t11*t14*t15;
    *p++ = 0.0;
    *p++ = d4 * t2 + d6 * t46 + d5 * (t7*t19 + t11 * t42) + a2 * t3*t14 + a3 * t3*t5*t14 - a3 * t4*t6*t14;
    *p++ = -t2 * t28;
    *p++ = t2 * t31;
    *p++ = t2 * t40;
    *p++ = d6 * t14*t16 - d6 * t2*t3*t5*t7*t15 + d6 * t2*t4*t6*t7*t15 + d6 * t2*t3*t4*t11*t15 + d6 * t2*t5*t6*t11*t15;
    *p++ = 0.0;
    *p++ = 0.0;
    *p++ = t35 + t38 + t39 + a2 * t3 - d6 * t33*0.5;
    *p++ = t35 + t38 + t39 - d6 * t33*0.5;
    *p++ = t38 + t39 - d6 * t33*0.5;
    *p++ = -d6 * t15*t27;
    *p++ = 0.0;
    *p++ = 0.0;
    *p++ = t2;
    *p++ = t2;
    *p++ = t2;
    *p++ = t14 * t27;
    *p++ = t46;
    *p++ = 0.0;
    *p++ = -t14;
    *p++ = -t14;
    *p++ = -t14;
    *p++ = t2 * t27;
    *p++ = -t47 - t51;
    *p++ = 1.0;
    *p++ = 0.0;
    *p++ = 0.0;
    *p++ = 0.0;
    *p++ = -t26;
    *p++ = -t16 * t27;
}




