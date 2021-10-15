#ifndef UR_KINEMATICS_H
#define UR_KINEMATICS_H

#include <math.h>
#include <vector>
#include <array>

#include "math_calculate.h"
#include "matrix_inverse.h"
#include "ur_datatype.h"

class UrKinematics 
{
public:
	void FK(ur_data_type::Joint joint, ur_data_type::CartPose &pose, double R06_33[3][3] = {});
	bool IK(ur_data_type::CartPose pose, ur_data_type::Joint joint_ref, ur_data_type::Joint &joint_target);
	void getRotationByJoint(ur_data_type::Joint joint,  double R06_33[3][3]);
	void calibrateWorkpiece(double transition_wp2base[4][4]);
private:
	MatrixInverse44 matrix_inverse_44_;

	void mutationJudge(ur_data_type::Joint joint_start, ur_data_type::Joint& joint_mirror);

	void urForward(const double q[], std::array<double, 6>& tcp, double R06_33[3][3]);
	bool urInverse(double tcp_pose[6],double joint_ref[6], double joint_target[6], double error_min);
#if 0
	int Matrix44UrTool2TCP(double tool[6], double homo_matrix[][4]);
	int Euler2RMatrix_XYZ(double euler[], double rotXYZ[][3]);
	int Matrix44Tool2Camera(double tool2camera[6], double homo_matrix[][4]);
	int rotx(double theta, double rot_x[][3]);
	int roty(double theta, double rot_y[][3]);
	int rotz(double theta, double rot_z[][3]);
	std::vector<double> UpperArmKinemeticForward(float*body_para, const std::vector<double> theta, int angle_num = 7);
	std::array<double,6> UpperArmInverse(const float *body_para, double* T_goat, double* joint_old);

	void GetBaseWpTcp(double tool_vector[6], double tcp_tem[6], double tcp2wp[6], double T_work_piece2b[4][4]);

	void TCP2Workpiece(double T_Wp2B[][4], double tcp2b[], double tcp2workpiece[]);
	void PoseWp2Base(double trans_mat[][4], FVector2F& vector_in, int game_type=0);

	int WorkpieceCalibrate(double T_Wp2B[][4]);
#endif
};

#endif
