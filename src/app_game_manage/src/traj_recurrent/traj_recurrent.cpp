#include <fstream>
#include <unistd.h> 

#include "path_file.h"
#include "thread_pool.h"
#include "traj_recurrent.h"

using namespace ur_data_type;


TrajRecurrent::TrajRecurrent()
{
	params_.repeated_count = 0;
	passive_speed_ = 0;

	repeat_times_ = 0;
	speed_max_ = 0.1;
	sleep_count_ = 0;

	cart_path_file_ = CART_PATH_FILE_DIR;
	joint_path_file_ = JOINT_PATH_FILE_DIR;

	is_processing_ = false;
	is_recurring_ = false;
	running_state_ = app_game_model::IDEL;

	wait_loop_times_ = 0;
	wait_loop_count_ = WAIT_LOOP_COUNT;
	joint_path_.clear();
}

TrajRecurrent::~TrajRecurrent()
{

}

bool TrajRecurrent::initParams(app_game_model::AppGameParams params)
{
	if (is_processing_)
	{
        printf("TrajRecurrent: game is running!\n");
	    return true;
	}
    params_.need_record = params.need_record;

    params_.training_mode = params.training_mode; //params_.training_mode
    params_.force_level = params.force_level;
    params_.rigidity_level = params.rigidity_level;
    params_.apply_force_mode = params.apply_force_mode;
    params_.speed_level = params.speed_level;
    params_.repeated_count = params.repeated_count;

    params_.sensor_info.is_sensor_used = params.sensor_info.is_sensor_used;
    if (params_.sensor_info.is_sensor_used)
    {
        params_.sensor_info.remove_zero_bias = params.sensor_info.remove_zero_bias;
        params_.sensor_info.sensor_ip = params.sensor_info.sensor_ip;
    }

	passive_speed_ = static_cast<double>(params_.speed_level) / 3.0;
	speed_max_ *= static_cast<double>(params_.speed_level) / 3.0;
	printf("repeat_count = %d, speed_max = %f\n", params_.repeated_count, speed_max_);

	if (!generate_path_.generateJointPathFromFile(joint_path_file_.c_str()))
	{
		printf("TrajRecurrent: read joint path failed!\n");
		return false;
	}

    joint_path_ = generate_path_.getJointPath();

	if (!generateJointPathFromJointFile())
	{
		printf("TrajRecurrent: generate joint path failed from joint file!\n");
		return false;
	}
	Joint joint = generate_path_.getJointEndPosition();
	printf("TrajRecurrent: end joint: %lf, %lf, %lf, %lf, %lf, %lf\n",
		joint.jVal[0], joint.jVal[1], joint.jVal[2], joint.jVal[3], joint.jVal[4], joint.jVal[5]);

	updateWaitLoopCount(params_.speed_level);
	return true;
}

bool TrajRecurrent::start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor)
{
	if (is_processing_)
	{
        printf("TrajRecurrent: game is running!\n");
	    return true;
	}

    if(control_arm_rt == nullptr)
    {
        printf("TrajRecurrent: start failed, control_arm_rt is nullptr!\n");
		return false;  
    }

    robot_rt_ = control_arm_rt;

	if (!moveToEndJoint(control_arm_rt->getActualJoint()))
	{
		printf("TrajRecurrent: move to end_pose failed!\n");
		return false;
	}
    printf("TrajRecurrent: moving to end joint position.");
	while(!isReachedJointPosition(control_arm_rt->getActualJoint(), generate_path_.getJointEndPosition()))
	{
        printf(".");
	    usleep(500000);
	}
    printf("reached end position.\n");

	running_state_ = app_game_model::WAITING;
	is_processing_ = true;

	ThreadPool::getInstance().registerHandler(this, THREAD_INDEX);
    printf("TrajRecurrent: force level = %d, speed_level = %d, repeated_count = %d\n",
        params_.force_level, params_.speed_level, params_.repeated_count);
	printf("TrajRecurrent is running!\n");
	return true;
}

void TrajRecurrent::stop()
{
	if (!is_processing_) return;

	if (!robot_rt_->initGlobalSpeed())
	{
        printf("Warning: TrajRecurrent initial robot global speed failed.\n");
	}

	is_processing_ = false;
	running_state_ = app_game_model::IDEL;
	ThreadPool::getInstance().deregisterHandler(THREAD_INDEX);
	joint_path_.clear();
	printf("TrajRecurrent: stoped.\n");
}

void TrajRecurrent::process() 
{
	Joint joint;
	if (!is_processing_) return;

	switch (running_state_)
	{
	case app_game_model::PROCESSING:
	{
		if (isReachedJointPosition(robot_rt_->getActualJoint(), generate_path_.getJointEndPosition()))
		{
			printf("第%d次复现开始... ...\n", repeat_times_);
			moveFromEndToStart();
			wait_loop_times_ = 0;
			running_state_ = app_game_model::WAITING;
			return;
		}

		if (isReachedJointPosition(robot_rt_->getActualJoint(), generate_path_.getJointStartPosition()))
		{
			moveFromStartToEnd();
			wait_loop_times_ = 0;
			running_state_ = app_game_model::WAITING;
			printf("第%d次复现即将结束... ...\n", repeat_times_);
		}
	}
	break;
	case app_game_model::WAITING:
	{
		Joint actual_joint = robot_rt_->getActualJoint();
		Joint end_joint = generate_path_.getJointEndPosition();

		if (isReachedJointPosition(actual_joint, end_joint))
		{
            if (repeat_times_ >= 1 && wait_loop_times_ < wait_loop_count_)
            {
                wait_loop_times_++;
                return;
            }

			if (sleep_count_ > 100)
			{
			    printf("终点验证完毕，已到终点。\n");
				running_state_ = app_game_model::REPLAYING;
				sleep_count_ = 0;
				return;
			}
			else
			{
				sleep_count_++;
				//printf("正在进行终点验证，请稍后... ...\n");
			}
		}
		if (isReachedJointPosition(actual_joint, generate_path_.getJointStartPosition()))
		{
            if (wait_loop_times_ < wait_loop_count_)
            {
                wait_loop_times_++;
                return;
            }

			if (sleep_count_ > 100)
			{
                printf("起点验证完毕，已到起点。\n");
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
			is_recurring_ = true;
			running_state_ = app_game_model::PROCESSING;
			printf("轨迹复现：总共需要复现 %d 次，下面进行第 %d 次复现：\n", params_.repeated_count, repeat_times_);
		}
	}
	break;
	default:;
	}
	
	//usleep(100000); // 10ms
}


void TrajRecurrent::pause()
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

void TrajRecurrent::replay()
{
#if 0
	running_state_ = app_game_model::REPLAYING;
#endif
}


std::string TrajRecurrent::getName() 
{
    return "TrajRecurrent";
}


void TrajRecurrent::encodeRuntimeInfo(char* buf, int &buf_size)
{
    return;
}

bool TrajRecurrent::decodeRuntimeCmd(char* buf, int buf_size)
{
    return true;
}
