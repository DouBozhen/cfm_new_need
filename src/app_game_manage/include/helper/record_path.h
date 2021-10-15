#ifndef RECORD_PATH_H
#define RECORD_PATH_H

#include <string>
#include <array>
#include <vector>

#include "ur_datatype.h"
#include "control_arm_rt.h"
#include "ur_kinematics.h"
#include "math_calculate.h"

#include "handler.h"

#define RECORD_PATH_THREAD_INDEX 1

#define P2P_LIMIT_MAX_CART 0.2
#define P2P_LIMIT_MAX_JOINT 0.5

struct RecordLimit
{
	float cart_p2p_distance_min;
	float joint_p2p_distance_min;
};

class RecordPath : public Handler
{
public:
	RecordPath();
    ~RecordPath();

    void setRecordLimit(RecordLimit limit);

	bool start(
		ControlArmRealtime* robot_rt, 
		struct RecordLimit limit,
		std::string cart_pose_file,
		std::string cart_pose_file_impendance,
		std::string joint_file,
		std::string joint_file_impendance
		);
	void setFiles(
		ControlArmRealtime* robot_rt, 
		std::string cart_pose_file,
		std::string cart_pose_file_impendance,
		std::string joint_file,
		std::string joint_file_impendance
	);
	virtual void process() override;
	void stop();
	void stopRecordEndPositions();
	bool isRunning();

	bool updateEndPosition();



    virtual std::string getName() override;
private:
    RecordLimit record_limit_;
	int thread_index_;
	bool is_running_;
	int sleep_count_;

	UrKinematics ur_kine_;

	ControlArmRealtime* robot_rt_;
	std::string cart_pose_file_;
	std::string cart_pose_file_impendance_;
	std::string joint_file_;
	std::string joint_file_impendance_;

	std::vector<ur_data_type::CartPose> cart_path_;
	std::vector<ur_data_type::Joint> joint_path_;
	std::vector<ur_data_type::CartPose> cart_path_impendance_;
	std::vector<ur_data_type::Joint> joint_path_impendance_;

	ur_data_type::CartPose cart_pose_;
	ur_data_type::CartPose cart_pose_last_;
	ur_data_type::Joint joint_;
	ur_data_type::Joint joint_last_;

	int impendance_count_;

	void getJointPath(ur_data_type::Joint joint, int impendance_count);
	void recordJointPath(std::string file);
	void recordJointPathImpendance(std::string file);
	bool addEndPositionInJointPath(ur_data_type::Joint joint);
	bool addEndPositionInJointPathImpendance(ur_data_type::Joint joint);
	
	bool addEndingPositionsInJointPath(std::string file);
	bool addEndingPositionsInJointPathImpendance(std::string file);

	void getCartPath(ur_data_type::CartPose pose, int impendance_count);
	void recorCartPath(std::string file);
	void recordCartPathImpendance(std::string file);
	bool addEndPositionInCartPath(ur_data_type::CartPose pose);
	bool addEndPositionInCartPathImpendance(ur_data_type::CartPose pose);

	bool addEndingPositionsInCartPath(std::string file);
	bool addEndingPositionsInCartPathImpendance(std::string file);

	bool getLastJointPositionFomeFile(std::string file, std::array<double, 6> &position);
};

#endif

