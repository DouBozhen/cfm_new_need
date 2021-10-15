#include <stdio.h>
#include "force_control.h"
using namespace ur_data_type;
using namespace force_data_type;

int main()
{
    ForceControl force_cotrol = ForceControl();
    Joint joint;
    joint.jVal[0] = 1.524270;
    joint.jVal[1] = -1.652888;
    joint.jVal[2] = 1.666815;
    joint.jVal[3] = -1.513228;
    joint.jVal[4] = -2.098298;
    joint.jVal[5] = 0.783472;

    CartPose pose;
    pose.point.x = 0.047354; 
    pose.point.y = -0.449512;  
    pose.point.z = 0.429698;
    pose.rpy.rx = -1.032243; 
    pose.rpy.ry = -2.402758;  
    pose.rpy.rz = 0.165565;

    ForceValue force_error;
    force_error.fx = -1.039934;
    force_error.fy = 0.584326;
    force_error.fz = -3.120785;

    // force_cotrol.boundaryProtection(pose,joint, force_error);
    return 0;
}

#if 0
    void boundaryProtection(
        const ur_data_type::CartPose tcp_target, 
        const ur_data_type::Joint actual_joint,
        force_data_type::ForceValue& force_error
    );
#endif

// -1.039934, 0.584326, -3.120785
// -0.983084, 0.607871, -3.149343
// -0.014703, -0.008783, -0.054824
// -0.570058, 0.229077, -2.107047

// actual_joint: 1.524270, -1.652888, 1.666815, -1.513228, -2.098298, 0.783472
// actual_tcp: 0.047354, -0.449512, 0.429698, -1.032243, -2.402758, 0.165565