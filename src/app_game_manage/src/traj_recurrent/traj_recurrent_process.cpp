#include <fstream>
#include <math.h>
#include "traj_recurrent.h"

//#define TRAJ_RECORD

using namespace ur_data_type;
std::ofstream traj_recurr("../../../docs/traj_recurr.csv", fstream::out|ios_base::trunc);

bool TrajRecurrent::generateJointPathFromJointFile()
{
    /* end to start*/
	int path_position_count = joint_path_.size();
	for (size_t i = 0; i < path_position_count; ++i)
	{
		joint_path_.push_back(joint_path_[path_position_count - 1 - i]);
		printf("joint positon: %lf, %lf, %lf, %lf, %lf, %lf\n",
			joint_path_[path_position_count - 1 - i].jVal[0], joint_path_[path_position_count - 1 - i].jVal[1], joint_path_[path_position_count - 1 - i].jVal[2], 
			joint_path_[path_position_count - 1 - i].jVal[3], joint_path_[path_position_count - 1 - i].jVal[4], joint_path_[path_position_count - 1 - i].jVal[5]);
	}
	return true;
}

bool TrajRecurrent::moveToEndJoint(ur_data_type::Joint actual_joint)
{
	if (joint_path_.size() < 1)
	{
		printf("TrajRecurrent: joint path is null\n");
		return false;
	}

	if (isReachedJointPosition(actual_joint, generate_path_.getJointEndPosition()))
	{
		printf("TrajRecurrent: reached end joint position\n");
		return true;
	}

	Joint joint = generate_path_.getJointEndPosition();
	printf("end joint: %lf, %lf, %lf, %lf, %lf, %lf\n",
		joint.jVal[0], joint.jVal[1], joint.jVal[2], 
		joint.jVal[3], joint.jVal[4], joint.jVal[5]);
	if(!robot_rt_->jointMove(generate_path_.getJointEndPosition(), 0.5, 0.5, 3.0, 0.0))
	{
		printf("TrajRecurrent: move to end joint position failed.\n");
		return false;
	}
    printf("TrajRecurrent: jointMove success\n");
	return true;
}

bool TrajRecurrent::isReachedJointPosition(ur_data_type::Joint actual_joint, ur_data_type::Joint desire_joint)
{
	for (int j = 0; j != 6; ++j)
	{
		if (abs(actual_joint.jVal[j] - desire_joint.jVal[j]) > 0.15)
			return false;
	}

	return true;
}

bool TrajRecurrent::moveFromEndToStart()
{
	std::vector<ur_data_type::Joint> joint_position_list = generate_path_.getJointPath();

	if (joint_position_list.empty())
	{
		printf("TrajRecurrent: joint path is null\n");
		return true;
	}

	vector<Joint> joint_path_end2start;
	for (int index = 0; index != joint_position_list.size(); ++index)
	{
		joint_path_end2start.push_back(joint_path_[joint_position_list.size() - 1 - index]);
	}

	string script;
	robot_rt_->generateServoJScript(joint_path_end2start, abs(passive_speed_) / 2.0, abs(passive_speed_), script);
	if (!robot_rt_->sendScript(script))
	{
		printf("TrajRecurrent: sendScript failed\n");
		return false;
	}

	return true;

#if 0
	if (joint_path_.size() == 0)
	{
		printf("TrajRecurrent: joint path is null\n");
		return true;
	}
	printf("moveFromEndToStart: joint path size = %d\n", joint_path_.size());
	vector<Joint> joint_path_end2start;
	for (int index = 0; index != joint_path_.size(); ++index)
	{
		joint_path_end2start.push_back(joint_path_[index]);
	}
#endif
}

bool TrajRecurrent::moveFromStartToEnd()
{
	std::vector<ur_data_type::Joint> joint_position_list = generate_path_.getJointPath();
	if (joint_position_list.empty())
	{
		printf("TrajRecurrent: joint path is null\n");
		return true;
	}
	string script;
	robot_rt_->generateServoJScript(joint_position_list, abs(passive_speed_) / 2.0, passive_speed_, script);
	if (!robot_rt_->sendScript(script))
	{
		printf("TrajRecurrent: sendScript failed\n");
		return false;
	}
	return true;

#if 0
	if (joint_path_.size() == 0)
	{
		printf("TrajRecurrent: joint path is null\n");
		return true;
	}
	printf("moveFromStartToEnd: joint path size = %d\n", joint_path_.size());
	vector<Joint> joint_path_start2end;
	for (int index = joint_path_.size() - 1; index >= 0; --index)
	{
		joint_path_start2end.push_back(joint_path_[index]);
	}
#endif
}

void TrajRecurrent::updateWaitLoopCount(int speed_level)
{
    wait_loop_count_ = 400;
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
