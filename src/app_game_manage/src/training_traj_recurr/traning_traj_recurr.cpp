#include <fstream>
#include <unistd.h> 

#include "path_file.h"
#include "thread_pool.h"
#include "training_traj_recurr.h"

using namespace ur_data_type;


TrainingTrajRecurr::TrainingTrajRecurr()
{
	is_recurring_ = false;
	passive_speed_ = 0;
	speed_max_ = 0.1;

	repeat_times_ = 0;
	sleep_count_ = 0;

	robot_rt_ = nullptr;
	generate_path_proximal_ = GeneratePath();
	generate_path_distal_ = GeneratePath();

	memset(&pose_curr_, 0, sizeof(pose_curr_));
	joint_path_all_.clear();
	joint_path_proximal_.clear();
	joint_path_distal_.clear();

	cart_path_file_proximal_ = TRAINGING_CART_PATH_PROXIMAL_DIR;
	joint_path_file_proximal_ = TRAINGING_JOINT_PATH_PROXIMAL_DIR;
	cart_path_file_distal_ = TRAINGING_CART_PATH_DISTAL_DIR;
	joint_path_file_distal_ = TRAINGING_JOINT_PATH_DISTAL_DIR;

	wait_loop_times_ = 0;
	wait_loop_count_ = WAIT_LOOP_COUNT;

	is_recurring_ = false;
	running_state_ = app_game_model::IDEL;
	remove_zero_bias_ = false;
	training_mode_ = 0;
	memset(&params_, 0, sizeof(params_));
}

TrainingTrajRecurr::~TrainingTrajRecurr()
{
	joint_path_proximal_.clear();
	joint_path_distal_.clear();
	joint_path_all_.clear();
}

bool TrainingTrajRecurr::initParams(app_game_model::AppGameParams params)
{
	if (is_recurring_)
	{
        printf("TrainingTrajRecurr: game is running!\n");
	    return true;
	}

	initGameParams(params);

	if (!generatePathProximal(joint_path_file_proximal_.c_str())
		|| !generatePathDistal(joint_path_file_distal_.c_str()))
	{
		printf("TrainingTrajRecurr: read joint path failed!\n");
		return false;
	}

	if (!generatePathAll())
	{
		printf("TrainingTrajRecurr: generate all joint path failed!\n");
		return false;
	}

	joint_home_ = joint_path_proximal_[0];
	joint_distal_limit_ = joint_path_distal_[joint_path_distal_.size() - 1];
	joint_proximal_limit_ = joint_path_proximal_[joint_path_proximal_.size() - 1];

	updateWaitLoopCount(params_.speed_level);
	return true;
}

void TrainingTrajRecurr::setComponent(ControlArmRealtime* control_arm_rt)
{
	if(control_arm_rt == nullptr)
    {
        printf("TrainingTrajRecurr: start failed, control_arm_rt is nullptr!\n");
		return;  
    }

    robot_rt_ = control_arm_rt;
}

bool TrainingTrajRecurr::start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor)
{
	if (is_recurring_)
	{
        printf("TrainingTrajRecurr: game is running!\n");
	    return true;
	}

    if(control_arm_rt == nullptr)
    {
        printf("TrainingTrajRecurr: start failed, control_arm_rt is nullptr!\n");
		return false;  
    }

    robot_rt_ = control_arm_rt;

	if (!moveFromHomeToDistalLimit())
	{
		printf("TrainingTrajRecurr: move to end_pose failed!\n");
		return false;
	}

	printf("joint_distal_limit_: %lf, %lf, %lf, %lf, %lf, %lf\n",
		joint_distal_limit_.jVal[0],
		joint_distal_limit_.jVal[1],
		joint_distal_limit_.jVal[2],
		joint_distal_limit_.jVal[3],
		joint_distal_limit_.jVal[4],
		joint_distal_limit_.jVal[5]
		);

    printf("TrainingTrajRecurr: moving to distal limit position.");
	while(!isReachedDistalLimitPosition())
	{
        printf(".");
	    usleep(100000);
	}
    printf("reached distal limit position.\n");

	running_state_ = app_game_model::WAITING;
	is_recurring_ = true;

	ThreadPool::getInstance().registerHandler(this, THREAD_INDEX);
	printf("TrainingTrajRecurr is running!\n");
	return true;
}

void TrainingTrajRecurr::stop()
{
	if (!is_recurring_) return;

	is_recurring_ = false;
	repeat_times_ = 0;
	running_state_ = app_game_model::IDEL;
	ThreadPool::getInstance().deregisterHandler(THREAD_INDEX);

	if (!isReachedHomePosition())
	{
		printf("TrainingTrajRecurr: moving to home position.");
		if (moveDistalLimitToHome(passive_speed_))
		{
			while(!isReachedHomePosition())
			{
				printf(".");
				usleep(100000);
			}
			printf("reached home position.\n");
		}
		else 
		{
			printf("Warning: Failed move to proximal limit position\n");
		}
		usleep(100000);
	}

	if (!robot_rt_->initGlobalSpeed())
	{
        printf("Warning: TrainingTrajRecurr initial robot global speed failed.\n");
	}

	printf("TrainingTrajRecurr: stoped.\n");
}

void TrainingTrajRecurr::process()
{
	Joint joint;
	if (!is_recurring_) return;

	switch (running_state_)
	{
	case app_game_model::PROCESSING:
	{
		if (isReachedDistalLimitPosition())
		{
			printf("第%d次复现开始... ...\n", repeat_times_);
			moveFromDistalToProximal();
			wait_loop_times_ = 0;
			running_state_ = app_game_model::WAITING;
			return;
		}

		if (isReachedProximalLimitPosition())
		{
			moveFromProximalToDistal();
			wait_loop_times_ = 0;
			running_state_ = app_game_model::WAITING;
			printf("第%d次复现即将结束... ...\n", repeat_times_);
		}
	}
	break;
	case app_game_model::WAITING:
	{
		if (isReachedDistalLimitPosition())
		{
            if (repeat_times_ >= 1 && wait_loop_times_ < wait_loop_count_)
            {
                wait_loop_times_++;
                return;
            }

			if (sleep_count_ > 100)
			{
			    printf("起点验证完毕，已到起点。\n");
				running_state_ = app_game_model::REPLAYING;
				sleep_count_ = 0;
				return;
			}
			else
			{
				sleep_count_++;
				//printf("正在进行起点验证，请稍后... ...\n");
			}
		}
		if (isReachedProximalLimitPosition())
		{
            if (wait_loop_times_ < wait_loop_count_)
            {
                wait_loop_times_++;
                return;
            }

			if (sleep_count_ > 100)
			{
                printf("终点验证完毕，已到终点。\n");
				running_state_ = app_game_model::PROCESSING;
				sleep_count_ = 0;
			}
			else
			{
				sleep_count_++;
			}
		}
	}
	break;
	case app_game_model::STOPING:
	{
		robot_rt_->stopJointMove();
		running_state_ = app_game_model::IDEL;
	}
	break;
	case app_game_model::REPLAYING:
	{
		if (repeat_times_ > params_.repeated_count - 1)
		{
			printf("轨迹复现结束.\n");
			running_state_ = app_game_model::STOPING;
		}
		else
		{
			repeat_times_++;
			// is_recurring_ = true;
			running_state_ = app_game_model::PROCESSING;
			printf("轨迹复现：总共需要复现 %d 次，下面进行第 %d 次复现：\n", params_.repeated_count, repeat_times_);
		}
	}
	break;
	default:;
	}
	
	//usleep(100000); // 10ms
}

void TrainingTrajRecurr::pause()
{
#if 0
	if (running_state_ == app_game_model::WAITING)
	{
		running_state_ = app_game_model::PROCESSING;
	}
	else
	{
		running_state_ = app_game_model::WAITING;
	}
#endif
}

void TrainingTrajRecurr::replay()
{
#if 0
	running_state_ = app_game_model::REPLAYING;
#endif
}

std::string TrainingTrajRecurr::getName() 
{
    return "TrainingTrajRecurr";
}

bool TrainingTrajRecurr::isReachedHomePosition()
{
	Joint joint_vel = robot_rt_->getActualJointVel();
	return isReachedJointPosition(robot_rt_->getActualJoint(), joint_home_, joint_vel);
}

bool TrainingTrajRecurr::isReachedHomePosition(ur_data_type::Joint actual_joint, ur_data_type::Joint actual_joint_vel)
{
	return isReachedJointPosition(actual_joint, joint_home_, actual_joint_vel);
}

bool TrainingTrajRecurr::isReachedProximalLimitPosition()
{
	return isReachedJointPosition(robot_rt_->getActualJoint(), joint_proximal_limit_, robot_rt_->getActualJointVel());
}

bool TrainingTrajRecurr::isReachedDistalLimitPosition()
{
	return isReachedJointPosition(robot_rt_->getActualJoint(), joint_distal_limit_, robot_rt_->getActualJointVel());
}

ur_data_type::Joint TrainingTrajRecurr::getProximalLimitPosition()
{
	return joint_proximal_limit_;
}

ur_data_type::Joint TrainingTrajRecurr::getDistalLimitPosition()
{
 	return joint_distal_limit_;
}

ur_data_type::Joint TrainingTrajRecurr::getHomePosition()
{
	return joint_home_;
}

