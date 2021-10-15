#ifndef GNERATE_PATH_H
#define GNERATE_PATH_H

#include "ur_datatype.h"

class GeneratePath
{
public:
    GeneratePath();
	bool generateCartPathFromFile(const char* file);
	bool generateJointPathFromFile(const char* file);

    std::vector<ur_data_type::CartPose> getCartPath() { return cart_pose_list_;};
    std::vector<ur_data_type::Joint> getJointPath()
	{
		printf("joint_position_list_ .size = %d\n", joint_position_list_.size());
		return joint_position_list_;
	};

	inline ur_data_type::CartPose getCartStartPosition() { return pose_start_; };
	inline ur_data_type::Joint getJointStartPosition() { return joint_start_; };

	inline ur_data_type::CartPose getCartEndPosition() { return pose_end_; };
	inline ur_data_type::Joint getJointEndPosition() { return joint_end_; };

private:
    ur_data_type::CartPose pose_start_;
    ur_data_type::Joint joint_start_;
    ur_data_type::CartPose pose_end_;
    ur_data_type::Joint joint_end_;

	std::vector<ur_data_type::CartPose> cart_pose_list_;
	std::vector<ur_data_type::Joint> joint_position_list_;
};

#endif