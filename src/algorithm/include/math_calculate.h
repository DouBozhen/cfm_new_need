#ifndef MATH_CALCULATE_H
#define MATH_CALCULATE_H

#include <vector>
#include <array>

#include <gsl/gsl_matrix_double.h>
#include <gsl/gsl_vector_double.h>
#include <gsl/gsl_eigen.h>

#include <math.h>
#include "planar_datatype.h"
#include "ur_datatype.h"

#define PI (3.1415926)
#define DOUBLE_ZERO (0.000000001)

namespace math_calculate 
{
    void matrixMul16(const double vec_a[16], const double vec_b[16], double result[16]);
    void matrixMul44(const double mat_a[4][4], const double mat_b[4][4], double result[4][4]);
    void matrixMul33(double mat_a[3][3], double mat_b[3][3], double result[3][3]);
    double radToDegree(double rad);
    double degreeToRad(double degree);
    double getVecendrialAngle(const planar_data_type::PlanarPoint origin, const planar_data_type::PlanarPoint start, const planar_data_type::PlanarPoint end);
    double getP2PLength(const planar_data_type::PlanarPoint start, const planar_data_type::PlanarPoint end);
    double getP2PLength(const ur_data_type::CartesianPoint start, const ur_data_type::CartesianPoint end);
    void matrixTranspose33(const double src[3][3], double des[3][3]);
    void getRotationFromTran(const double tran[4][4], double rot[3][3]);
    void rpy2Rotation(double rpy[], double rot_zyx[3][3]);
    void rotation2Rpy(double rot[3][3], double rpy[3]);
    void rotVector2Rpy(double rot_vector[3], double rpy[3]);
    int rpy2RotVector(double rpy[3], double r_vector[3]);
    int pose2Transition(ur_data_type::CartPose pose, double homo_matrix[][4]);

    void getPlanarPoseByTransition();
    void getPoseInWpToBase(double trans_mat[][4], planar_data_type::PlanarPoint &point); //UrKinematics::PoseWp2Base
    void getTransitionTcp2Base(ur_data_type::CartPose tcp_pose, double trans[4][4]);
    double getPlanarP2PDistance(planar_data_type::PlanarPoint start, planar_data_type::PlanarPoint end);
    double getP2PDistance(ur_data_type::CartesianPoint start, ur_data_type::CartesianPoint end);

    void vectorTransTcp(const double* vector, double rotation[3][3], int num, double* tcp); //VectorTransBaseTool
    void vectorTransBase(const double* vector, double rotation[3][3], int num, double* base); //VectorTransToolBase

    std::array<double, 3>  cartPoint2DoubleList(const ur_data_type::CartesianPoint point);

    void solveEign(double mat[36], std::vector<double>& eign_value);
    bool areaIntegral(
        planar_data_type::PlanarPoint buttom_left,
        planar_data_type::PlanarPoint buttom_right,
        planar_data_type::PlanarPoint top_left,
        planar_data_type::PlanarPoint top_right   
    );

    double limitAngleRange(double angle);

    ur_data_type::Joint getJointDelta(ur_data_type::Joint start, ur_data_type::Joint end);
    double getJointDeltaMax(ur_data_type::Joint start, ur_data_type::Joint end);

    void scaleRpyDistanceP2P(ur_data_type::CartRpy start, ur_data_type::CartRpy& end);
    double maxJointDistanceP2P(ur_data_type::Joint start, ur_data_type::Joint end);
    ur_data_type::CartPose doubleArrayConvertToCartesianPose(std::array<double, 6> pose);
}


#endif
