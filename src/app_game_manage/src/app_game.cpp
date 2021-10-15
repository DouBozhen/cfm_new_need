#include <cstring>
#include "app_game.h"
#include "ur_datatype.h"
#include "math_calculate.h"

using namespace ur_data_type;
using namespace math_calculate;

void AppGame::generateCmdJointSpeed(ur_data_type::Joint joint_vel,
    ur_data_type::Joint speed, force_data_type::FTSensorData vw,
    bool &remove_zero_bias, ur_data_type::Joint &cmd_speed, double &acc)
{
    Joint delta = getJointDelta(speed, joint_vel);

    double delta_sum_abs_square = 0.0;
    for (int j = 0; j != 6; ++j)
    {
        delta_sum_abs_square += delta.jVal[j] * delta.jVal[j];
    }
    double  delta_sum_abs = sqrt(delta_sum_abs_square);
    double acc_temp = 300 * delta_sum_abs + 0.5;
    if (remove_zero_bias)
    {
        cmd_speed = speed;
        acc = acc_temp;
    }
    else
    {
        if (pow(vw.force.fx, 2) + pow(vw.force.fy, 2) + pow(vw.force.fz, 2) > acc_temp)
        {
            memset(cmd_speed.jVal, 0, sizeof(cmd_speed.jVal));
            acc = 0.5;
            remove_zero_bias = true;
        }
        else 
        {
            /* Velocity and acceleration remain constant */
        }
    }
}
