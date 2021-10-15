#ifndef UR_DATATYPE_H
#define UR_DATATYPE_H

#include <string>

namespace ur_data_type
{
    enum {JOINT_NUM = 6};

    struct CartesianPoint
    {
        double x;
        double y;
        double z;
    };

    struct CartRpy
    {
        double rx;
        double ry;
        double rz;
    };

    struct CartPose
    {
        CartesianPoint point;
        CartRpy rpy;
    };

    struct Joint
    {
        double jVal[JOINT_NUM];
    };

    struct DHParam
    {
        ur_data_type::Joint  theta;
        ur_data_type::Joint  a;
        ur_data_type::Joint  d;
        ur_data_type::Joint  alpha;
    };
}

#endif
