#include <fstream>
#include <math.h>
#include "training_traj_recurr.h"

//#define TRAJ_RECORD

using namespace ur_data_type;

void TrainingTrajRecurr::initGameParams(app_game_model::AppGameParams params)
{
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
}

bool TrainingTrajRecurr::generatePathProximal(const char* file)
{
	if (!generate_path_proximal_.generateJointPathFromFile(file))
	{
		printf("TrainingTrajRecurr: read joint path failed!\n");
		return false;
	}
	joint_path_proximal_.clear();
    joint_path_proximal_ = generate_path_proximal_.getJointPath();
	return true;
}

bool TrainingTrajRecurr::generatePathDistal(const char* file)
{
	if (!generate_path_distal_.generateJointPathFromFile(file))
	{
		printf("TrainingTrajRecurr: read joint path failed!\n");
		return false;
	}
	joint_path_distal_.clear();
    joint_path_distal_ = generate_path_distal_.getJointPath();
	return true;
}

bool TrainingTrajRecurr::generatePathAll()
{
	joint_path_all_.clear();

	vector<ur_data_type::Joint>::reverse_iterator riter;
	vector<ur_data_type::Joint> joint_path_distal_reverse;

	for (riter = joint_path_distal_.rbegin(); riter != joint_path_distal_.rend(); riter++)
    {
		joint_path_distal_reverse.push_back(*riter);
	}

	if (!joint_path_distal_reverse.empty())
	{
		joint_path_all_.assign(joint_path_distal_reverse.begin(), joint_path_distal_reverse.end());
	}

	if (joint_path_proximal_.empty())
	{
		return true;
	}

	std::vector<ur_data_type::Joint> joint_path_proximal = joint_path_proximal_;
	vector<ur_data_type::Joint>::iterator first_it = joint_path_proximal.begin();
	joint_path_proximal.erase(first_it);

	if (!joint_path_proximal.empty())
	{
		joint_path_all_.insert(joint_path_all_.end(), joint_path_proximal.begin(), joint_path_proximal.end());
	}

	if (!trajFitting())
	{
		printf("TrainingTrajRecurr: fitting trajectory failed\n");
		return false;
	}
	printf("joint_path_all_.size() = %d\n", joint_path_all_.size());
	for (int i = 0; i != joint_path_all_.size(); ++i)
	{
		printf("%d: %lf, %lf, %lf, %lf, %lf, %lf\n",
			joint_path_all_[i].jVal[0],
			joint_path_all_[i].jVal[1],
			joint_path_all_[i].jVal[2],
			joint_path_all_[i].jVal[3],
			joint_path_all_[i].jVal[4],
			joint_path_all_[i].jVal[5]
			);
	}

	return true;
}

bool TrainingTrajRecurr::moveFromHomeToProximalLimit()
{
	if (joint_path_proximal_.empty())
	{
		printf("TrainingTrajRecurr: proximal path is null\n");
		return true;
	}

	string script;
	robot_rt_->generateServoJScript(joint_path_proximal_, abs(passive_speed_) / 2.0, abs(passive_speed_), script);
	if (!robot_rt_->sendScript(script))
	{
		printf("TrainingTrajRecurr: sendScript failed\n");
		return false;
	}

	return true;
}

#if 0
bool TrainingTrajRecurr::moveFromActualToHome()
{

}
#endif

bool TrainingTrajRecurr::moveFromHomeToDistalLimit()
{
	if (joint_path_distal_.empty())
	{
		printf("TrainingTrajRecurr: proximal path is null\n");
		return true;
	}

	string script;
	robot_rt_->generateServoJScript(joint_path_distal_, abs(passive_speed_) / 2.0, abs(passive_speed_), script);
	
	if (!robot_rt_->sendScript(script))
	{
		printf("TrainingTrajRecurr: sendScript failed\n");
		return false;
	}

	return true;
}

bool TrainingTrajRecurr::moveProximalLimitToHome(double vel)
{
#if 0
	if (!generate_path_proximal_.generateJointPathFromFile(joint_path_file_proximal_.c_str()))
	{
		printf("TrainingTrajRecurr: read proximal joint path failed!\n");
		return false;
	}
	joint_path_proximal_ = generate_path_proximal_.getJointPath();

	if (joint_path_proximal_.empty())
	{
		printf("TrainingTrajRecurr: proximal path is null\n");
		return true;
	}

	joint_home_ = joint_path_proximal_[0];
	joint_proximal_limit_ = joint_path_proximal_[joint_path_proximal_.size() - 1];
#endif
	std::vector<ur_data_type::Joint> joint_path_proximal_reverse;
	vector<ur_data_type::Joint>::reverse_iterator riter;

	for (riter = joint_path_proximal_.rbegin(); riter != joint_path_proximal_.rend(); riter++)
    {
		joint_path_proximal_reverse.push_back(*riter);
	}

	string script;
	robot_rt_->generateServoJScript(joint_path_proximal_reverse, abs(vel) / 2.0, abs(vel), script);
	if (!robot_rt_->sendScript(script))
	{
		printf("TrainingTrajRecurr: sendScript failed\n");
		return false;
	}

	return true;
}

int TrainingTrajRecurr::generateProximalPath()
{
	if (!generate_path_proximal_.generateJointPathFromFile(joint_path_file_proximal_.c_str()))
	{
		printf("TrainingTrajRecurr: read proximal joint path failed!\n");
		return -1;
	}
	std::vector<ur_data_type::Joint> path = generate_path_proximal_.getJointPath();

	joint_path_proximal_ = generate_path_proximal_.getJointPath();

	if (joint_path_proximal_.size() == 0)
	{
		printf("TrainingTrajRecurr: proximal path is null\n");
		return 0;
	}

	joint_home_ = joint_path_proximal_[0];
	joint_proximal_limit_ = joint_path_proximal_[joint_path_proximal_.size() - 1];
	return joint_path_proximal_.size();
}

int TrainingTrajRecurr::generateDistalPath()
{
	if (!generate_path_distal_.generateJointPathFromFile(joint_path_file_distal_.c_str()))
	{
		printf("TrainingTrajRecurr: read distal joint path failed!\n");
		return -1; /* generate failed */
	}
	joint_path_distal_ = generate_path_distal_.getJointPath();

	if (joint_path_distal_.empty())
	{
		printf("TrainingTrajRecurr: distal path is null\n");
		return 0;
	}

	joint_home_ = joint_path_distal_[0];
	joint_distal_limit_ = joint_path_distal_[joint_path_distal_.size() - 1];
	return joint_path_distal_.size();
}

bool TrainingTrajRecurr::moveDistalLimitToHome(double vel)
{
#if 0
	if (!generate_path_distal_.generateJointPathFromFile(joint_path_file_distal_.c_str()))
	{
		printf("TrainingTrajRecurr: read distal joint path failed!\n");
		return false;
	}
	joint_path_distal_ = generate_path_distal_.getJointPath();

	if (joint_path_distal_.empty())
	{
		printf("TrainingTrajRecurr: distal path is null\n");
		return true;
	}

	joint_home_ = joint_path_distal_[0];
	joint_distal_limit_ = joint_path_distal_[joint_path_distal_.size() - 1];
#endif
	std::vector<ur_data_type::Joint> joint_path_distal_reverse;
	vector<ur_data_type::Joint>::reverse_iterator riter;

	for (riter = joint_path_distal_.rbegin(); 
		 riter != joint_path_distal_.rend(); 
		 riter++)
    {
        joint_path_distal_reverse.push_back(*riter);
    }

	string script;
	robot_rt_->generateServoJScript(joint_path_distal_reverse, abs(vel) / 2.0, abs(vel), script);
	
	if (!robot_rt_->sendScript(script))
	{
		printf("TrainingTrajRecurr: sendScript failed\n");
		return false;
	}

	return true;
}

bool TrainingTrajRecurr::moveFromDistalToProximal()
{
	if (joint_path_all_.empty())
	{
		printf("TrainingTrajRecurr: path is null\n");
		return false;
	}

	string script;
	robot_rt_->generateServoJScript(joint_path_all_, abs(passive_speed_) / 2.0, abs(passive_speed_), script);
	
	if (!robot_rt_->sendScript(script))
	{
		printf("TrainingTrajRecurr: sendScript failed\n");
		return false;
	}

	return true;
}

bool TrainingTrajRecurr::moveFromProximalToDistal()
{
	if (joint_path_all_.empty())
	{
		printf("TrainingTrajRecurr: path is null\n");
		return false;
	}

	std::vector<ur_data_type::Joint> joint_path_all_reverse;
	vector<ur_data_type::Joint>::reverse_iterator riter;

	for (riter = joint_path_all_.rbegin(); riter != joint_path_all_.rend(); riter++)
    {
        joint_path_all_reverse.push_back(*riter);
    }

	string script;
	robot_rt_->generateServoJScript(joint_path_all_reverse, abs(passive_speed_) / 2.0, abs(passive_speed_), script);
	if (!robot_rt_->sendScript(script))
	{
		printf("TrainingTrajRecurr: sendScript failed\n");
		return false;
	}

	return true;
}

bool TrainingTrajRecurr::isReachedJointPosition(
	ur_data_type::Joint actual_joint, 
	ur_data_type::Joint desire_joint, 
	ur_data_type::Joint actual_joint_speed)
{
	if (abs(actual_joint.jVal[0] - desire_joint.jVal[0]) < 0.2
		&& abs(actual_joint.jVal[1] - desire_joint.jVal[1]) < 0.2
		&& abs(actual_joint.jVal[2] - desire_joint.jVal[2]) < 0.2
		&& abs(actual_joint.jVal[3] - desire_joint.jVal[3]) < 0.2
		&& abs(actual_joint.jVal[4] - desire_joint.jVal[4]) < 0.2
		&& abs(actual_joint.jVal[5] - desire_joint.jVal[5]) < 0.2
		&& actual_joint_speed.jVal[0] < 0.0001
		&& actual_joint_speed.jVal[1] < 0.0001
		&& actual_joint_speed.jVal[2] < 0.0001
		&& actual_joint_speed.jVal[3] < 0.0001
		&& actual_joint_speed.jVal[4] < 0.0001
		&& actual_joint_speed.jVal[5] < 0.0001
		)
			return true;

	return false;
}

void TrainingTrajRecurr::updateWaitLoopCount(int speed_level)
{
	wait_loop_count_ = 100;
#if 0
	switch(speed_level)
	{
		case app_game_model::SPEED_LEVEL_SLOW:
			wait_loop_count_ = 500;
		break;
		case app_game_model::SPEED_LEVEL_MEDIUM:
			wait_loop_count_ = 400;
		break;
		case app_game_model::SPEED_LEVEL_FAST:
			wait_loop_count_ = 300;
		break;
		default:
		    wait_loop_count_ = 300;
	};
#endif
}

bool TrainingTrajRecurr::trajFitting()
{
	/* to do...... */
	return true;
}