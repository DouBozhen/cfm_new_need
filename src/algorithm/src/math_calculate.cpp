#include <assert.h>
#include "math_calculate.h"

namespace math_calculate{

void matrixMul16(const double vec_a[16], const double vec_b[16], double result[16])
{
    result[0] = vec_a[0] * vec_b[0] + vec_a[1] * vec_b[4] + vec_a[2] * vec_b[8] + vec_a[3] * vec_b[12];
    result[1] = vec_a[0] * vec_b[1] + vec_a[1] * vec_b[5] + vec_a[2] * vec_b[9] + vec_a[3] * vec_b[13];
    result[2] = vec_a[0] * vec_b[2] + vec_a[1] * vec_b[6] + vec_a[2] * vec_b[10] + vec_a[3] * vec_b[14];
    result[3] = vec_a[0] * vec_b[3] + vec_a[1] * vec_b[7] + vec_a[2] * vec_b[11] + vec_a[3] * vec_b[15];

    result[4] = vec_a[4] * vec_b[0] + vec_a[5] * vec_b[4] + vec_a[6] * vec_b[8] + vec_a[7] * vec_b[12];
    result[5] = vec_a[4] * vec_b[1] + vec_a[5] * vec_b[5] + vec_a[6] * vec_b[9] + vec_a[7] * vec_b[13];
    result[6] = vec_a[4] * vec_b[2] + vec_a[5] * vec_b[6] + vec_a[6] * vec_b[10] + vec_a[7] * vec_b[14];
    result[7] = vec_a[4] * vec_b[3] + vec_a[5] * vec_b[7] + vec_a[6] * vec_b[11] + vec_a[7] * vec_b[15];

    result[8] = vec_a[8] * vec_b[0] + vec_a[9] * vec_b[4] + vec_a[10] * vec_b[8] + vec_a[11] * vec_b[12];
    result[9] = vec_a[8] * vec_b[1] + vec_a[9] * vec_b[5] + vec_a[10] * vec_b[9] + vec_a[11] * vec_b[13];
    result[10] = vec_a[8] * vec_b[2] + vec_a[9] * vec_b[6] + vec_a[10] * vec_b[10] + vec_a[11] * vec_b[14];
    result[11] = vec_a[8] * vec_b[3] + vec_a[9] * vec_b[7] + vec_a[10] * vec_b[11] + vec_a[11] * vec_b[15];

    result[12] = vec_a[12] * vec_b[0] + vec_a[13] * vec_b[4] + vec_a[14] * vec_b[8] + vec_a[15] * vec_b[12];
    result[13] = vec_a[12] * vec_b[1] + vec_a[13] * vec_b[5] + vec_a[14] * vec_b[9] + vec_a[15] * vec_b[13];
    result[14] = vec_a[12] * vec_b[2] + vec_a[13] * vec_b[6] + vec_a[14] * vec_b[10] + vec_a[15] * vec_b[14];
    result[15] = vec_a[12] * vec_b[3] + vec_a[13] * vec_b[7] + vec_a[14] * vec_b[11] + vec_a[15] * vec_b[15];
}

void matrixMul44(const double mat_a[4][4], const double mat_b[4][4], double result[4][4])
{
    result[0][0] = mat_a[0][0] * mat_b[0][0] + mat_a[0][1] * mat_b[1][0] + mat_a[0][2] * mat_b[2][0] + mat_a[0][3] * mat_b[3][0];
    result[0][1] = mat_a[0][0] * mat_b[0][1] + mat_a[0][1] * mat_b[1][1] + mat_a[0][2] * mat_b[2][1] + mat_a[0][3] * mat_b[3][1];
    result[0][2] = mat_a[0][0] * mat_b[0][2] + mat_a[0][1] * mat_b[1][2] + mat_a[0][2] * mat_b[2][2] + mat_a[0][3] * mat_b[3][2];
    result[0][3] = mat_a[0][0] * mat_b[0][3] + mat_a[0][1] * mat_b[1][3] + mat_a[0][2] * mat_b[2][3] + mat_a[0][3] * mat_b[3][3];

    result[1][0] = mat_a[1][0] * mat_b[0][0] + mat_a[1][1] * mat_b[1][0] + mat_a[1][2] * mat_b[2][0] + mat_a[1][3] * mat_b[3][0];
    result[1][1] = mat_a[1][0] * mat_b[0][1] + mat_a[1][1] * mat_b[1][1] + mat_a[1][2] * mat_b[2][1] + mat_a[1][3] * mat_b[3][1];
    result[1][2] = mat_a[1][0] * mat_b[0][2] + mat_a[1][1] * mat_b[1][2] + mat_a[1][2] * mat_b[2][2] + mat_a[1][3] * mat_b[3][2];
    result[1][3] = mat_a[1][0] * mat_b[0][3] + mat_a[1][1] * mat_b[1][3] + mat_a[1][2] * mat_b[2][3] + mat_a[1][3] * mat_b[3][3];

    result[2][0] = mat_a[2][0] * mat_b[0][0] + mat_a[2][1] * mat_b[1][0] + mat_a[2][2] * mat_b[2][0] + mat_a[2][3] * mat_b[3][0];
    result[2][1] = mat_a[2][0] * mat_b[0][1] + mat_a[2][1] * mat_b[1][1] + mat_a[2][2] * mat_b[2][1] + mat_a[2][3] * mat_b[3][1];
    result[2][2] = mat_a[2][0] * mat_b[0][2] + mat_a[2][1] * mat_b[1][2] + mat_a[2][2] * mat_b[2][2] + mat_a[2][3] * mat_b[3][2];
    result[2][3] = mat_a[2][0] * mat_b[0][3] + mat_a[2][1] * mat_b[1][3] + mat_a[2][2] * mat_b[2][3] + mat_a[2][3] * mat_b[3][3];

    result[3][0] = mat_a[3][0] * mat_b[0][0] + mat_a[3][1] * mat_b[1][0] + mat_a[3][2] * mat_b[2][0] + mat_a[3][3] * mat_b[3][0];
    result[3][1] = mat_a[3][0] * mat_b[0][1] + mat_a[3][1] * mat_b[1][1] + mat_a[3][2] * mat_b[2][1] + mat_a[3][3] * mat_b[3][1];
    result[3][2] = mat_a[3][0] * mat_b[0][2] + mat_a[3][1] * mat_b[1][2] + mat_a[3][2] * mat_b[2][2] + mat_a[3][3] * mat_b[3][2];
    result[3][3] = mat_a[3][0] * mat_b[0][3] + mat_a[3][1] * mat_b[1][3] + mat_a[3][2] * mat_b[2][3] + mat_a[3][3] * mat_b[3][3];
}

void matrixMul33(double mat_a[3][3], double mat_b[3][3], double result[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            result[i][j] = 0;
            for (int k = 0; k < 3; k++)
            {
                result[i][j] += mat_a[i][k] * mat_b[k][j];
            }
        }
    }
}

double radToDegree(double rad)
{
    return rad * 180 / M_PI;
}

double degreeToRad(double degree)
{
    return degree * M_PI / 180;
}

double getVecendrialAngle(
    const planar_data_type::PlanarPoint origin,
    const planar_data_type::PlanarPoint start, 
    const planar_data_type::PlanarPoint end)
{
    float angle = atan2(start.x-origin.x, start.y-origin.y) - atan2(end.x-origin.x, end.y-origin.y);
    if (angle > M_PI) angle -= 2 * M_PI;
    if (angle < -M_PI) angle += 2 * M_PI;
    return radToDegree(angle);
}
 
double getP2PLength(
    const planar_data_type::PlanarPoint start, 
    const planar_data_type::PlanarPoint end)
{
    return sqrt(
        (end.x - start.x)*(end.x - start.x)
        + (end.y - start.y)*(end.y - start.y)
    );
}

double getP2PLength(
    const ur_data_type::CartesianPoint start, 
    const ur_data_type::CartesianPoint end)
{
    return sqrt(
        (end.x - start.x)*(end.x - start.x)
        + (end.y - start.y)*(end.y - start.y)
        + (end.z - start.z)*(end.z - start.z)
    );
}

void matrixTranspose33(const double src[3][3], double des[3][3])
{
    for (int i = 0; i < 3; ++i)
    {
        for (int j = 0; j < 3; ++j)
        {
            des[i][j] = src[j][i];
        }
    }
}

void getRotationFromTran(const double tran[4][4], double rot[3][3])
{
    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rot[i][j] = tran[i][j];
        }
    }
}

void rpy2Rotation(double rpy[], double rot_zyx[3][3])
{
	double  rot_x[3][3] = { {1 ,0, 0}, {0, cos(rpy[0]), -sin(rpy[0])},{ 0 ,sin(rpy[0]), cos(rpy[0])} };
    double  rot_y[3][3] = { {cos(rpy[1]) ,0 ,sin(rpy[1])}, {0 ,1, 0}, {-sin(rpy[1]), 0,cos(rpy[1])} };
    double  rot_z[3][3] = { {cos(rpy[2]), -sin(rpy[2]) ,0},{ sin(rpy[2]) ,cos(rpy[2]), 0}, {0 ,0 ,1} };

    double rot_zy[3][3];

    for (int m = 0; m < 3; m++)
    {
        for (int s = 0; s < 3; s++)
        {
            rot_zy[m][s] = 0;
            for (int n = 0; n < 3; n++)
            {
                rot_zy[m][s] += rot_z[m][n] * rot_y[n][s];
            }
        }
    }

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            rot_zyx[i][j] = 0;
            for (int k = 0; k < 3; k++)
            {
                rot_zyx[i][j] += rot_zy[i][k] * rot_x[k][j];
            }
        }
    }
}


void rotation2Rpy(double rot[3][3], double rpy[3])
{
    double  sy = sqrt(rot[0][0] * rot[0][0] + rot[1][0] * rot[1][0]);

    if (sy > 1e-6)
    {
        rpy[0] = atan2(rot[2][1], rot[2][2]);
        rpy[1] = atan2(-rot[2][0], sy);
        rpy[2] = atan2(rot[1][0], rot[0][0]);
    }
    else
    {
        if (rot[2][0] < 0)//beta=90
        {
            rpy[0] = atan2(rot[0][1], rot[1][1]);
            rpy[1] = M_PI / 2;
            rpy[2] = 0;
        }
        else
        {
            rpy[0] = -atan2(rot[0][1], rot[1][1]);
            rpy[1] = -M_PI / 2;
            rpy[2] = 0;
        }
    } 
}

void rotVector2Rpy(double rot_vector[3], double rpy[3])
{
    double cb;
    double theta = sqrt(rot_vector[0] * rot_vector[0] + rot_vector[1] * rot_vector[1] + rot_vector[2] * rot_vector[2]);
    double kx = rot_vector[0] / theta;
    double ky = rot_vector[1] / theta;
    double kz = rot_vector[2] / theta;
    double cth = cos(theta);
    double sth = sin(theta);
    double vth = 1 - cos(theta);
    double r11 = kx * kx*vth + cth;
    double r12 = kx * ky*vth - kz * sth;
    //double r13 = kx*kz*vth + ky*sth;
    double r21 = kx * ky*vth + kz * sth;
    double r22 = ky * ky*vth + cth;
    //double r23 = ky*kz*vth - kx*sth;
    double r31 = kx * kz*vth - ky * sth;
    double r32 = ky * kz*vth + kx * sth;
    double r33 = kz * kz*vth + cth;
    rpy[1] = atan2(-r31, sqrt(r11*r11 + r21 * r21));
    if (rpy[1] > 89.99*M_PI / 180)
    {
        rpy[1] = 89.99*M_PI / 180;
        rpy[2] = 0;
        rpy[0] = atan2(r12, r22);
    }
    else if (rpy[1] < -89.99*M_PI / 180)
    {
        rpy[1] = -89.99*M_PI / 180;
        rpy[2] = 0;
        rpy[0] = -atan2(r12, r22);
    }
    else
    {
        cb = cos(rpy[1]);
        rpy[2] = atan2(r21 / cb, r11 / cb);
        rpy[0] = atan2(r32 / cb, r33 / cb);
    }
}



int rpy2RotVector(double rpy[3], double r_vector[3])
{
    double ca = cos(rpy[2]);
    double cb = cos(rpy[1]);
    double cg = cos(rpy[0]);
    double sa = sin(rpy[2]);
    double sb = sin(rpy[1]);
    double sg = sin(rpy[0]);
    double r11 = ca * cb;
    double r12 = ca * sb*sg - sa * cg;
    double r13 = ca * sb*cg + sa * sg;
    double r21 = sa * cb;
    double r22 = sa * sb*sg + ca * cg;
    double r23 = sa * sb*cg - ca * sg;
    double r31 = -sb;
    double r32 = cb * sg;
    double r33 = cb * cg;
    double theta = acos((r11 + r22 + r33 - 1) / 2);
    double sth = sin(theta);
    double kx = (r32 - r23) / (2 * sth);
    double ky = (r13 - r31) / (2 * sth);
    double kz = (r21 - r12) / (2 * sth);
    r_vector[0] = theta * kx;
    r_vector[1] = theta * ky;
    r_vector[2] = theta * kz;
    return 0;
}

int pose2Transition(ur_data_type::CartPose pose, double homo_matrix[][4])
{
    // homo_matrix 是齐次变换矩阵4×4
    double P[3];
    double R1[3][3];
    double num[4] = { 0,0,0,1 };

    P[0] = pose.point.x;
    P[1] = pose.point.y;
    P[2] = pose.point.z;

    double tcp_pose1[3] = { pose.rpy.rx, pose.rpy.ry, pose.rpy.rz};
    printf("tcp_pose1: %lf, %lf, %lf\n", tcp_pose1[0], tcp_pose1[1], tcp_pose1[2]);
    double rpy[3];
    rotVector2Rpy(tcp_pose1, rpy);
    rpy2Rotation(rpy, R1);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            homo_matrix[i][j] = R1[i][j];
        }
    }

    for (int i = 0; i < 3; i++)
    {
        homo_matrix[i][3] = P[i];
    }
    for (int i = 0; i < 4; i++)
    {

        homo_matrix[3][i] = num[i];
    }
    return 0;
}

double getPlanarDistanceP2P(planar_data_type::PlanarPoint start, planar_data_type::PlanarPoint end)
{
    return sqrt((end.x - start.x)*(end.x - start.x) + (end.y - start.y)*(end.y - start.y));
}

double getP2PDistance(ur_data_type::CartesianPoint start, ur_data_type::CartesianPoint end)
{
    return sqrt((end.x - start.x)*(end.x - start.x) 
        + (end.y - start.y)*(end.y - start.y) 
        + (end.z - start.z)*(end.z - start.z)); 
}


void vectorTransTcp(const double* vector, double rotation[3][3], int num, double* tcp)
{
	if (num == 3)
	{
		for (int i = 0; i < 3; i++)
		{
			tcp[i] = 0;
			for (int j = 0; j < 3; j++)
			{
				tcp[i] += vector[j] * rotation[j][i];
			}
		}
	}
	else
	{
		assert(num == 6);
		for (int i = 0; i < 3; i++)
		{
			tcp[i] = 0; 
            tcp[i + 3] = 0;
			for (int j = 0; j < 3; j++)
			{
				tcp[i] += vector[j] * rotation[j][i];
				tcp[i + 3] += vector[j + 3] * rotation[j][i];
			}
		}
	}
}

void vectorTransBase(const double* vector, double rotation[3][3], int num, double* base)
{
	if (num == 3)
	{
		for (int i = 0; i < 3; i++)
		{
			base[i] = 0;
			for (int j = 0; j < 3; j++)
			{
				base[i] += vector[j] * rotation[i][j];
			}
		}
	}
	else
	{
		assert(num == 6);
		for (int i = 0; i < 3; i++)
		{
			base[i] = 0;
            base[i + 3] = 0;

			for (int j = 0; j < 3; j++)
			{
				base[i] += vector[j] * rotation[i][j];
				base[i + 3] += vector[j + 3] * rotation[i][j];
			}
		}
	} 
}


std::array<double, 3> cartPoint2DoubleList(const ur_data_type::CartesianPoint point)
{
    std::array<double, 3> double_point = {point.x, point.y, point.z};
    return double_point;
}

void solveEign(double mat[36], std::vector<double>& eign_value)
{
	gsl_matrix_view gsl_mat = gsl_matrix_view_array(mat, 6, 6);
	gsl_vector *eval = gsl_vector_alloc(6);
	gsl_matrix *evec = gsl_matrix_alloc(6, 6);
	gsl_eigen_symmv_workspace *gsl_ws = gsl_eigen_symmv_alloc(6);
	gsl_eigen_symmv(&gsl_mat.matrix, eval, evec, gsl_ws);
	gsl_eigen_symmv_free(gsl_ws);

	gsl_eigen_symmv_sort(eval, evec, GSL_EIGEN_SORT_ABS_ASC);
	double eigval_min = gsl_vector_get(eval, 0);
	eign_value.push_back(eigval_min);

	gsl_vector_view evec_i = gsl_matrix_column(evec, 0);
	for (int i = 0; i < 6; i++)
	{
		eign_value.push_back(evec_i.vector.data[i * 6]);
	}

	gsl_vector_free(eval);
	gsl_matrix_free(evec);
}

double getPlanarP2PDistance(planar_data_type::PlanarPoint start, planar_data_type::PlanarPoint end)
{
    return sqrt((end.x-start.x)*(end.x-start.x) + (end.y-start.y)*(end.y-start.y));
}

bool areaIntegral(
    planar_data_type::PlanarPoint buttom_left,
    planar_data_type::PlanarPoint buttom_right,
    planar_data_type::PlanarPoint top_left,
    planar_data_type::PlanarPoint top_right,
    double &area   
)
{
    double length_left = getPlanarP2PDistance(buttom_left, top_left);
	double length_right = getPlanarP2PDistance(buttom_right, top_right);
	double high = getPlanarP2PDistance(buttom_left, buttom_right);

	double area_temp = (length_left + length_right)*high / 2;
	if (area_temp > 10e-5)
    {
        area = area_temp;
        return true;
    }
    
    area = 0.0;
    return false;
}

double limitAngleRange(double angle)
{
   	double angle_limited = 0;
	if (angle > 3.14)
	{
		angle_limited = angle - 6.28;
	}
	else if (angle < -3.14)
	{
		angle_limited = 6.28 + angle;
	}
	else
	{
		angle_limited = angle;
	}
	return angle_limited; 
}

ur_data_type::Joint getJointDelta(ur_data_type::Joint start, ur_data_type::Joint end)
{
    ur_data_type::Joint delta;
    for (int j = 0; j != 6; ++j)
    {
        delta.jVal[j] = end.jVal[j] - start.jVal[j];
    }
    return delta;
}

double getJointDeltaMax(ur_data_type::Joint start, ur_data_type::Joint end)
{
    double max_delta = 0.0;
    for (int j = 0; j != 6; ++j)
    {
        double delta = fabs(end.jVal[j] - start.jVal[j]);
        max_delta  = max_delta < delta ? delta : max_delta;
    }
    return max_delta;
}


void getTransitionTcp2Base(ur_data_type::CartPose tcp_pose, double trans[4][4])
{
    double tcp_rpy[] = {tcp_pose.rpy.rx, tcp_pose.rpy.ry, tcp_pose.rpy.rz};
    double rpy[3];
    rotVector2Rpy(tcp_rpy, rpy);

    double rotation[3][3];
    rpy2Rotation(rpy, rotation);

    for (int i = 0; i < 3; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            trans[i][j] = rotation[i][j];
        }
    }

    trans[0][3] = tcp_pose.point.x;
    trans[1][3] = tcp_pose.point.y;
    trans[2][3] = tcp_pose.point.z;   

    trans[3][0] = 0.0;
    trans[3][1] = 0.0;
    trans[3][2] = 0.0;
    trans[3][3] = 1.0;   
}

void getPoseInWpToBase(double trans_mat[][4], planar_data_type::PlanarPoint &point)
{
    ur_data_type::CartPose tcp_in_wp;
    tcp_in_wp.point.x = point.x;
    tcp_in_wp.point.y = point.y;
    tcp_in_wp.point.z = 0.0;
    tcp_in_wp.rpy.rx = 0.0;
    tcp_in_wp.rpy.ry = 0.0;
    tcp_in_wp.rpy.rz = 0.0;

    double tcp_in_wp_16[4][4] = {};
    double tcp_in_base_16[4][4] = {};
    double rot_tcp_in_base[3][3] = {};
    double rpy_theta[3] = {};

    getTransitionTcp2Base(tcp_in_wp, tcp_in_wp_16);
    matrixMul44(trans_mat, tcp_in_wp_16, tcp_in_base_16);
    getRotationFromTran(tcp_in_base_16, rot_tcp_in_base);
    rotation2Rpy(rot_tcp_in_base, rpy_theta);

    point.x = tcp_in_base_16[0][3];
    point.y = tcp_in_base_16[1][3];
}

void scaleRpyDistanceP2P(ur_data_type::CartRpy start, ur_data_type::CartRpy& end)
{
	if (end.rx - start.rx < -PI)
	{
		end.rx += PI * 2;
	}
	if (end.rx - start.rx > PI)
	{
		end.rx -= PI * 2;
	}

	if (end.ry - start.ry < -PI)
	{
		end.ry += PI * 2;
	}
	if (end.ry - start.ry > PI)
	{
		end.ry -= PI * 2;
	}

	if (end.rz - start.rz < -PI)
	{
		end.rz += PI * 2;
	}
	if (end.rz - start.rz > PI)
	{
		end.rz -= PI * 2;
	}
}

double maxJointDistanceP2P(ur_data_type::Joint start, ur_data_type::Joint end)
{
	double max = 0.0;
	double distance = 0.0;
	for (int j = 0; j != 6; ++j)
	{
		distance = fabs(end.jVal[j] - start.jVal[j]);
		max = (distance > max) ? distance : max;
	}

	return max;
}

ur_data_type::CartPose doubleArrayConvertToCartesianPose(std::array<double, 6> pose)
{
	ur_data_type::CartPose cart_pose;
	cart_pose.point.x = pose[0];
	cart_pose.point.y = pose[1];
	cart_pose.point.z = pose[2];
	cart_pose.rpy.rx = pose[3];
	cart_pose.rpy.ry = pose[4];
	cart_pose.rpy.rz = pose[5];
	return cart_pose;
}

}