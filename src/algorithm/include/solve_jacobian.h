#ifndef SOLVE_JACOBIAN_H
#define SOLVE_JACOBIAN_H

#include "ur_datatype.h"
#include "math_calculate.h"
#include "ur_kinematics.h"
#include "force_datatype.h"

class SloveJacobian
{
public: 
    bool inverseJacobian(
        ur_data_type::Joint position,
        force_data_type::FTSensorData &vw, ur_data_type::Joint &speed
    );

    void jacobi(const double * v, double *J);

private:
    double par_a_[6] = { 0,-0.42500,-0.39225,0,0,0 }; //apar
	double par_d_[6] = { 0.089159,0,0,0.10915,0.09465,0.0823 }; // dpar
    UrKinematics ur_kine_;
    bool ALU(double a[6][6], double b[6], double res[6]);
};

#endif