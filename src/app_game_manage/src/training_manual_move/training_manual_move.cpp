#include <cstring>
#include <unistd.h> 
#include <fstream>
#include "ur_datatype.h"
#include "training_manual_move.h"
#include "math_calculate.h"
#include "thread_pool.h"
#include "path_file.h"

using namespace ur_data_type;
using namespace app_game_model;
using namespace force_data_type;
using namespace math_calculate;
using namespace std;

TrainingManualMove::TrainingManualMove()
{
    training_mode_ = NONE;
    running_state_ = IDEL;
    control_arm_rt_ = nullptr;
    ft_sensor_ = nullptr;
    remove_zero_bias_ = false;

    is_moving_ = false;
    memset(&params_, 0, sizeof(params_));

    memset(&start_joint_position_, 0, sizeof(start_joint_position_));
    memset(&end_joint_position_, 0, sizeof(end_joint_position_));

    record_path_limit_.cart_p2p_distance_min = 0.08;
    record_path_limit_.joint_p2p_distance_min = 0.03; /* 30mm */

    memset(&proximal_limit_position_, 0, sizeof(proximal_limit_position_));
    memset(&distal_limit_position_, 0, sizeof(distal_limit_position_));
    memset(&home_position_, 0, sizeof(home_position_));

    record_path_= RecordPath();
    manual_move_ = ManualMovement();
}

TrainingManualMove::~TrainingManualMove()
{

}

bool TrainingManualMove::initParams(app_game_model::AppGameParams params)
{
    params_.need_record = true;
    params_.training_move_type = params.training_move_type;
    params_.training_mode = params.training_mode;
    params_.force_level = FORCE_LEVEL_SMALL; //params.force_level;
    params_.rigidity_level = RIGIDITY_SMALL; //params.rigidity_level;
    params_.apply_force_mode = params.apply_force_mode;
    params_.speed_level = params.speed_level;
    params_.repeated_count = params.repeated_count;
    params_.need_record = params.need_record;
    params_.training_move_type = params.training_move_type;

    params_.sensor_info.is_sensor_used = params.sensor_info.is_sensor_used;
    if (params_.sensor_info.is_sensor_used)
    {
        params_.sensor_info.remove_zero_bias = params.sensor_info.remove_zero_bias;
        params_.sensor_info.sensor_ip = params.sensor_info.sensor_ip;
    }
    else 
    {
        params_.sensor_info.remove_zero_bias = true;
    }

    return true;
}

bool TrainingManualMove::start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor)
{
    if (is_moving_) return true;
    is_moving_ = true;

    if (control_arm_rt == nullptr || ft_sensor == nullptr)
    {
        printf("TrainingManualMove: start failed, pointer is nullptr\n");
        return false;
    }
    control_arm_rt_ = control_arm_rt;
    //usleep(1000000);

    Joint actual_joint = control_arm_rt_->getActualJoint();
    if(actual_joint.jVal[0] < DOUBLE_ZERO
        && actual_joint.jVal[1] < DOUBLE_ZERO
        && actual_joint.jVal[2] < DOUBLE_ZERO
        && actual_joint.jVal[3] < DOUBLE_ZERO
        && actual_joint.jVal[4] < DOUBLE_ZERO
        && actual_joint.jVal[5] < DOUBLE_ZERO
        )
    {
        printf("TrainingManualMove: start failed, control_arm_rt point error.\n");
        return false; 
    }

    ft_sensor_ = ft_sensor;
    FTSensorData sensor_data = ft_sensor_->getSensorData();
    printf("Warnning: TrainingManualMove: onceProcess: sensor_data %lf, %lf, %lf, %lf, %lf, %lf\n",
        sensor_data.force.fx, sensor_data.force.fy, sensor_data.force.fz,
        sensor_data.torque.tx, sensor_data.torque.ty, sensor_data.torque.tz);

    if (fabs(sensor_data.force.fx) < DOUBLE_ZERO
        && fabs(sensor_data.force.fy) < DOUBLE_ZERO
        && fabs(sensor_data.force.fz) < DOUBLE_ZERO
        && fabs(sensor_data.torque.tx) < DOUBLE_ZERO
        && fabs(sensor_data.torque.ty) < DOUBLE_ZERO
        && fabs(sensor_data.torque.tz) < DOUBLE_ZERO)
    {
        printf("TrainingManualMove: start ft sensor failed: data error.\n");
        return false;
    }

    if (!createManualMovement())
    {
        printf("TrainingManualMove: create ManualMovement failed.\n");
        return false;
    }

    if (!openRecordPath())
    {
        printf("TrainingManualMove: open RecordPath failed.\n");
        return false;
    }

    running_state_ = GameRunningState::IDEL;
 
    printf("TrainingManualMove: start successful.\n");
    return true;
}

void TrainingManualMove::pause()
{
    if (running_state_ == GameRunningState::WAITING)
	{
		running_state_ = GameRunningState::PROCESSING;
	}
	else
	{
		running_state_ = GameRunningState::WAITING;
	}
}

void TrainingManualMove::stop()
{
    if(!is_moving_)
    {
        printf("not is_moving_\n");
        return;
    } 

    is_moving_ = false;
    running_state_ = GameRunningState::STOPING;

#if 0
    // usleep(100000);
    // if (params_.sensor_info.remove_zero_bias)
    // {
        // ft_sensor_->removeBias();
        // usleep(100000);
    // }
#endif

    dropManualMovement();
    usleep(10000);

    closeRecordPath();
    usleep(10000);
}

void TrainingManualMove::replay() 
{
    running_state_ = GameRunningState::REPLAYING;
}

void TrainingManualMove::process() 
{
    return;
}

std::string TrainingManualMove::getName() 
{
    return "TrainingManualMove";
}

