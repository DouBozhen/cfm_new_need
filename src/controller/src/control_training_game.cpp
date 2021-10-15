#include <unistd.h>
#include "controller.h"
#include "app_game_model.h"
#include "force_datatype.h"
#include "math_calculate.h"
using namespace app_game_model;
using namespace math_calculate;
using namespace ur_data_type;

bool Controller::updateHomePosition()
{
    if(training_move_state_ == IDEL)
    {
        is_reached_position_ = false;
        training_manual_move_type_ = UPDATE_TRAINING_HOME_POSITION;
    }

    return true;
}

bool Controller::returnHomePosition()
{
    if (training_manual_move_type_ != TRAINING_NONE)
    {
        return false; 
    }

    int proximal_path_size = training_traj_recurr_.generateProximalPath();
    int distal_path_size = training_traj_recurr_.generateDistalPath();

    if (proximal_path_size == -1 && distal_path_size == -1)
    {
        printf("Controller: Error: generate path failed.\n");
        return false;
    }

    if (proximal_path_size == 0 && distal_path_size == 0)
    {
        printf("Controller: Error: paths is null.\n");
        return false;
    }

    Joint actual_joint = control_arm_rt_->getActualJoint();
    Joint actual_joint_vel = control_arm_rt_->getActualJointVel();

    if (training_traj_recurr_.isReachedHomePosition(actual_joint, actual_joint_vel))
    {
        training_manual_move_type_ = TRAINING_NONE;
        is_reached_position_ = true;
        return true;
    }

    training_manual_move_type_ = TRAINING_RETURN_HOME_POSITION;
    is_reached_position_ = false;

    if (proximal_path_size > 0)
    {
        proximal_limit_position_ = training_traj_recurr_.getProximalLimitPosition();
        home_position_ = training_traj_recurr_.getHomePosition();

        if (getJointDeltaMax(actual_joint, proximal_limit_position_) < JOINT_DELTA_MIN*3)
        {
            control_arm_rt_->initGlobalSpeed();
            training_traj_recurr_.setComponent(control_arm_rt_);
            return training_traj_recurr_.moveProximalLimitToHome(RETURN_HOME_SPEED_MAX);
        }
    }

    if (distal_path_size > 0)
    {
        distal_limit_position_ = training_traj_recurr_.getDistalLimitPosition();
        home_position_ = training_traj_recurr_.getHomePosition();

        if (getJointDeltaMax(actual_joint, distal_limit_position_) < JOINT_DELTA_MIN*53)
        {
            control_arm_rt_->initGlobalSpeed();
            training_traj_recurr_.setComponent(control_arm_rt_);
            return training_traj_recurr_.moveDistalLimitToHome(RETURN_HOME_SPEED_MAX);
        }
    }

    printf("Controller: can not move. position error : %d, %d\n", proximal_path_size, distal_path_size);
    return false;
}

#if 0
bool Controller::generateTrajProximal()
{
    Joint actual_joint = control_arm_rt_->getActualJoint();
    home_position_ = training_traj_recurr_.getHomePosition();
    printf("ready generate TrajProximal.\n");
    if(getJointDeltaMax(actual_joint, home_position_) < JOINT_DELTA_MIN*52)
    {
        printf("start generate TrajProximal.\n");
        if(training_move_state_ == IDEL)
        {
            is_reached_position_ = false;
            training_manual_move_type_ = TRAINING_GRNERATE_TRAJ_PROXIMAL;
        }

        return true;
    }
}

bool Controller::generateTrajDistal()
{
    Joint actual_joint = control_arm_rt_->getActualJoint();
    home_position_ = training_traj_recurr_.getHomePosition();
    if(getJointDeltaMax(actual_joint, home_position_) < JOINT_DELTA_MIN*3)
    {
        if(training_move_state_ == IDEL)
        {
            is_reached_position_ = false;
            training_manual_move_type_ = TRAINING_GRNERATE_TRAJ_DISTAL;
        }

        return true;
    }
}

bool Controller::updateEndPositionProximal()
{
    Joint actual_joint = control_arm_rt_->getActualJoint();
    proximal_limit_position_ = training_traj_recurr_.getProximalLimitPosition();
    if(getJointDeltaMax(actual_joint, proximal_limit_position_) < JOINT_DELTA_MIN*3)
    {
        if(training_move_state_ == IDEL)
        {
            is_reached_position_ = false;
            training_manual_move_type_ = TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL;
        }
        return true;
    }
}

bool Controller::updateEndPositionDistal()
{
    Joint actual_joint = control_arm_rt_->getActualJoint();
    distal_limit_position_ = training_traj_recurr_.getDistalLimitPosition();
    if(getJointDeltaMax(actual_joint, distal_limit_position_) < JOINT_DELTA_MIN*3)
    {
        if(training_move_state_ == IDEL)
        {
            is_reached_position_ = false;
            training_manual_move_type_ = TRAINING_UPDATE_LIMIT_POSITION_DISTAL;
        }
        return true;
    }
}
#endif

#if 1
bool Controller::generateTrajProximal()
{
    printf("start generate TrajProximal.\n");
    if(training_move_state_ == IDEL)
    {
        is_reached_position_ = false;
        training_manual_move_type_ = TRAINING_GRNERATE_TRAJ_PROXIMAL;
    }

    return true;
}

bool Controller::generateTrajDistal()
{
    if(training_move_state_ == IDEL)
    {
        is_reached_position_ = false;
        training_manual_move_type_ = TRAINING_GRNERATE_TRAJ_DISTAL;
    }
    return true;
}

bool Controller::updateEndPositionProximal()
{
    if(training_move_state_ == IDEL)
    {
        is_reached_position_ = false;
        training_manual_move_type_ = TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL;
    }
    return true;
}

bool Controller::updateEndPositionDistal()
{
    if(training_move_state_ == IDEL)
    {
        is_reached_position_ = false;
        training_manual_move_type_ = TRAINING_UPDATE_LIMIT_POSITION_DISTAL;
    }

    return true;
}
#endif

bool Controller::startTrainingTrajRercurr()
{
    if (training_traj_recurr_.isRunning())
    {
        return true;
    }
    #if 0
    if (!training_traj_recurr_.isReachedHomePosition())
    {
        app_game_model::AppGameParams params_;
        double passive_speed = static_cast<double>(params_.speed_level) / 3.0;
        if (training_traj_recurr_.isReachedDistalLimitPosition())
		{
            if(training_traj_recurr_.moveDistalLimitToHome(passive_speed))
            {
                return createTrainingTrajRecurr();
            }
		}
        if (training_traj_recurr_.isReachedDistalLimitPosition())
        {
            if(training_traj_recurr_.moveProximalLimitToHome(passive_speed))
            {
                return createTrainingTrajRecurr();
            }
        }  
        return false;
    }
    #endif

    #if 0
    Joint actual_joint = control_arm_rt_->getActualJoint();
    home_position_ = training_traj_recurr_.getHomePosition();
    if(actual_joint == home_position_)
    {
        return createTrainingTrajRecurr();
    }
    else
    {
        //from actual position move to home position
        TrainingTrajRecurr::moveFromActualToHome();
    }
    #endif
    return createTrainingTrajRecurr();
}

bool Controller::endTrainingTrajRecurr()
{  
    dropTrainingTrajRecurr();
}

bool Controller::isReachedHomePosition()
{
    return is_reached_position_;
}

bool Controller::isGenerateProximalTrajComplete()
{
    return is_reached_position_;
}

bool Controller::isReturnHomePosition()
{
    return is_reached_position_;
}

bool Controller::isGenerateDismalTrajComplete()
{
    return is_reached_position_;
}

bool Controller::createTrainingTrajRecurr()
{
    AppGameParams params;
    params.force_level = FORCE_LEVEL_MEDIUM;
    params.rigidity_level = RIGIDITY_SMALL;
    params.repeated_count = 3;
    params.speed_level = SPEED_LEVEL_SLOW;
    params.training_move_type = 0;
    params.apply_force_mode = Compliance;
    params.need_record = false;
    params.sensor_info.is_sensor_used = false;
    params.sensor_info.sensor_ip = ft_sensor_ip_;
    params.sensor_info.remove_zero_bias = true;

    if (!training_traj_recurr_.initParams(params))
    {
        printf("Controller: initial Training Traj Recurr failed.\n");
        return false;
    }

    if (!training_traj_recurr_.start(control_arm_rt_, &ft_sensor_))
    {
        printf("Controller: start Training Traj Recurr failed.\n");
        return false;
    }
    return true;
}

void Controller::dropTrainingTrajRecurr()
{
    if (training_traj_recurr_.isRunning())
    {
        training_traj_recurr_.stop();
    }
}

void Controller::trainingMoveStateMachine(bool sensor_status)
{
    switch (training_move_state_)
    {
        case IDLE:
        {
            if (sensor_status)
            {
                control_arm_rt_->initGlobalSpeed();
                training_move_state_ = TO_MOVE;
                gettimeofday(&start_move_time_, NULL);
                setLampBeltBlue();
                ft_sensor_.removeBias();
            }
        }
        break;
        case TO_MOVE:
        {
            if (!sensor_status)
            {
                struct timeval current_time;
                gettimeofday(&current_time, NULL);
 
                if (current_time.tv_sec - start_move_time_.tv_sec > 1.0
                    || current_time.tv_usec - start_move_time_.tv_usec > 500000)
                {
                    training_move_state_ = MOVING;
                    usleep(1000000);
                    createTrainingManualMove();
                }
                else
                {
                    training_move_state_ = IDLE;
                }

                setLampBeltGreen();
            }
        }
        break;
        case MOVING:
        {
            if (sensor_status)
            {
                training_move_state_ = TO_STOP;
                setLampBeltBlue();
            }
        }
        break;
        case TO_STOP:
        {
            if (!sensor_status)
            {
                dropTrainingManualMove();
                setLampBeltGreen();
                training_move_state_ = STOPED;
            }
        }
        break;
        case STOPED:
        {
            training_move_state_ = IDLE;
            is_reached_position_ = true;
            if (training_manual_move_type_ == TRAINING_GRNERATE_TRAJ_PROXIMAL)
            {
                proximal_limit_position_ = control_arm_rt_->getActualJoint();
            }

            if (training_manual_move_type_ == TRAINING_GRNERATE_TRAJ_DISTAL)
            {
                distal_limit_position_ = control_arm_rt_->getActualJoint();
            }

            training_manual_move_type_ = TRAINING_NONE;
            control_arm_rt_->initGlobalSpeed();
            ft_sensor_.removeBias();
        }
        break;
        default:;
    };
}

void Controller::trainingMoveStateMachineUpdateHome(bool sensor_status)
{
    switch (training_move_state_)
    {
        case IDLE:
        {
            if (sensor_status)
            {
                training_move_state_ = TO_MOVE;
                gettimeofday(&start_move_time_, NULL);
                control_arm_rt_->initGlobalSpeed();
                ft_sensor_.removeBias();
            }
        }
        break;
        case TO_MOVE:
        {
            if (sensor_status)
            {
                struct timeval current_time;
                gettimeofday(&current_time, NULL);

                if (current_time.tv_sec - start_move_time_.tv_sec > 1.0) /* 3s */
                {
                    training_move_state_ = MOVING;
                    createTrainingManualMove();
                    setLampBeltBlue();
                }
            }
        }
        break;
        case MOVING:
        {
            /* to start training manual move */
            if (!sensor_status)
            {
                dropTrainingManualMove();
                setLampBeltGreen();
                training_move_state_ = STOPED;
            }
        }
        break;
        case STOPED:
        {
            training_move_state_ = IDLE;
            is_reached_position_ = true;
            training_manual_move_type_ = TRAINING_NONE;
            home_position_ = control_arm_rt_->getActualJoint();
            control_arm_rt_->initGlobalSpeed();
            ft_sensor_.removeBias();
        }
        break;
        default:;
    };
}

void Controller::trainingReturnHome()
{
    static int loop_time = 0;
    if (loop_time < RETURN_HOME_WAIT_LOOP_NUM)
    {
        is_reached_position_ = false;
        loop_time++;
        return;
    }

    if (training_traj_recurr_.isReachedHomePosition())
    {
        control_arm_rt_->initGlobalSpeed();
        loop_time = 0;
        is_reached_position_ = true;
        training_manual_move_type_ = TRAINING_NONE;
    }

    printf("----------------- is return position: %d -----------\n", is_reached_position_); 
}

bool Controller::createTrainingManualMove()
{
    AppGameParams params;
    params.force_level = FORCE_LEVEL_MEDIUM;
    params.rigidity_level = RIGIDITY_SMALL;
    params.repeated_count = 3;
    params.speed_level = SPEED_LEVEL_SLOW;
    params.training_move_type = training_manual_move_type_;
    params.training_mode = PASSIVE;    
    params.apply_force_mode = Compliance;
    params.need_record = false;
    params.sensor_info.is_sensor_used = true;
    params.sensor_info.remove_zero_bias = true;
    params.sensor_info.sensor_ip = ft_sensor_ip_;

    training_manual_move_ = new TrainingManualMove();

    if (!training_manual_move_->initParams(params))
    {
        printf("Controller: initial Training Manual Move failed.\n");
        return false;
    }

    if (!training_manual_move_->start(control_arm_rt_, &ft_sensor_))
    {
        printf("Controller: start Training Manual Move failed.\n");
        return false;
    }

    return true;
}

void Controller::dropTrainingManualMove()
{
    if (training_manual_move_ != nullptr)
    {
        training_manual_move_->stop(); 

        delete training_manual_move_;
        training_manual_move_ = nullptr;
    }
}
