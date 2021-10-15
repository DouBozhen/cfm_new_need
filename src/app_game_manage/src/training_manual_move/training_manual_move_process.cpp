
#include <unistd.h>

#include "path_file.h"
#include "app_game_model.h"
#include "training_manual_move.h"

using namespace app_game_model;

bool TrainingManualMove::createManualMovement()
{
    AppGameParams params = params_;
    params.need_record = false;

    if(!manual_move_.initParams(params))
    {
        printf("TrainingManualMove: ManualMovement failed.\n");
        return false;  
    }

    if(!manual_move_.start(control_arm_rt_, ft_sensor_))
    {
        printf("Controller: start ManualMovement failed.\n");
        return false;  
    }

    return true;
}

void TrainingManualMove::dropManualMovement()
{
    if (manual_move_.isRunning())
    {
        manual_move_.stop();
        usleep(100000);
    }
}

bool TrainingManualMove::openRecordPath()
{
    switch (params_.training_move_type)
    {
        case TRAINING_GRNERATE_TRAJ_PROXIMAL:
        {
            cart_pose_file_ = TRAINGING_CART_PATH_PROXIMAL_DIR;
	        cart_pose_file_impendance_ = TRAINGING_CART_PATH_IMPENDANCE_PROXIMAL_DIR;
	        joint_file_ = TRAINGING_JOINT_PATH_PROXIMAL_DIR;
	        joint_file_impendance_ = TRAINGING_JOINT_PATH_IMPENDANCE_PROXIMAL_DIR;

            record_path_.setRecordLimit(record_path_limit_);
            proximal_limit_position_ = control_arm_rt_->getActualJoint();
            if (!record_path_.start(control_arm_rt_, record_path_limit_, cart_pose_file_, cart_pose_file_impendance_, joint_file_, joint_file_impendance_))
            {
                printf("TrainingManualMove: start RecordPath failed.\n");
                return false;
            }

            return true;
        }
        break;
        case TRAINING_GRNERATE_TRAJ_DISTAL:
        {
            cart_pose_file_ = TRAINGING_CART_PATH_DISTAL_DIR;
	        cart_pose_file_impendance_ = TRAINGING_CART_PATH_IMPENDANCE_DISTAL_DIR;
	        joint_file_ = TRAINGING_JOINT_PATH_DISTAL_DIR;
	        joint_file_impendance_ = TRAINGING_JOINT_PATH_IMPENDANCE_DISTAL_DIR;

            record_path_.setRecordLimit(record_path_limit_);
            distal_limit_position_ = control_arm_rt_->getActualJoint();
            if (!record_path_.start(control_arm_rt_, record_path_limit_, cart_pose_file_, cart_pose_file_impendance_, joint_file_, joint_file_impendance_))
            {
                printf("TrainingManualMove: start RecordPath failed.\n");
                return false;
            }

            return true;
        }
        break;
        case UPDATE_TRAINING_HOME_POSITION:
        {
            /* nothing to do. */
            return true;
        }
        break;
        case TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL:
        {
            cart_pose_file_ = TRAINGING_CART_PATH_PROXIMAL_DIR;
	        cart_pose_file_impendance_ = TRAINGING_CART_PATH_IMPENDANCE_PROXIMAL_DIR;
	        joint_file_ = TRAINGING_JOINT_PATH_PROXIMAL_DIR;
	        joint_file_impendance_ = TRAINGING_JOINT_PATH_IMPENDANCE_PROXIMAL_DIR;

            record_path_.setRecordLimit(record_path_limit_);
            proximal_limit_position_ = control_arm_rt_->getActualJoint();
            if (!record_path_.start(control_arm_rt_, record_path_limit_, cart_pose_file_, cart_pose_file_impendance_, joint_file_, joint_file_impendance_))
            {
                printf("TrainingManualMove: start RecordPath failed.\n");
                return false;
            }

            return true;    
        }
        break;
        case TRAINING_UPDATE_LIMIT_POSITION_DISTAL:
        {
            cart_pose_file_ = TRAINGING_CART_PATH_DISTAL_DIR;
	        cart_pose_file_impendance_ = TRAINGING_CART_PATH_IMPENDANCE_DISTAL_DIR;
	        joint_file_ = TRAINGING_JOINT_PATH_DISTAL_DIR;
	        joint_file_impendance_ = TRAINGING_JOINT_PATH_IMPENDANCE_DISTAL_DIR;

            record_path_.setRecordLimit(record_path_limit_);
            distal_limit_position_ = control_arm_rt_->getActualJoint();
            if (!record_path_.start(control_arm_rt_, record_path_limit_, cart_pose_file_, cart_pose_file_impendance_, joint_file_, joint_file_impendance_))
            {
                printf("TrainingManualMove: start RecordPath failed.\n");
                return false;
            }

            return true;
        }
        break;
        default:
        {
            printf("TrainingManualMove: open RecordPath failed. move type %d error\n", params_.training_move_type);
            return false;
        }
    }
}

void TrainingManualMove::closeRecordPath()
{
    printf("--------------------- training move type = %d -------------------\n", params_.training_move_type);
    switch (params_.training_move_type)
    {
        case TRAINING_GRNERATE_TRAJ_PROXIMAL:
        {
            if (record_path_.isRunning())
            {
                record_path_.stop();
            }
        }
        break;
        case TRAINING_GRNERATE_TRAJ_DISTAL:
        {
            if (record_path_.isRunning())
            {
                record_path_.stop();
            }
        }
        break;
        case UPDATE_TRAINING_HOME_POSITION:
        {
            /* nothing to do. */
        }
        break;
        case TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL:
        {
            if (record_path_.isRunning())
            {
                printf("----------- stop TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL ------------\n");
                record_path_.stopRecordEndPositions();
            }
        }
        break;
        case TRAINING_UPDATE_LIMIT_POSITION_DISTAL:
        {
            if (record_path_.isRunning())
            {
                printf("----------- stop TRAINING_UPDATE_LIMIT_POSITION_DISTAL ------------\n");
                record_path_.stopRecordEndPositions();
            }
        }
        break;
        default:;
    }
    usleep(100000);
}
