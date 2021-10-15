#ifndef TRAJ_RECURRENT_H
#define TRAJ_RECURRENT_H

#include <pthread.h>
#include <fstream>


#include "control_arm_rt.h"
#include "force_datatype.h"
#include "ft_sensor.h"
#include "app_game.h"
#include "force_control.h"
#include "ur_kinematics.h"
#include "solve_jacobian.h"
#include "generate_path.h"

#define WAIT_LOOP_COUNT (500)

class TrajRecurrent : public AppGame
{
public:
	TrajRecurrent();
	~TrajRecurrent();

	// static AppGameItf* CreateGame(JakaStateRT* jaka_robot);
	virtual bool initParams(app_game_model::AppGameParams params) override;
	virtual bool start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor) override;
	virtual void stop() override;
	virtual void pause() override;
	virtual void replay() override;
	virtual void process() override;
	virtual std::string getName() override;
	virtual bool isRunning() { return is_processing_;} 

	virtual void encodeRuntimeInfo(char* buf, int &buf_size) override;
    virtual bool decodeRuntimeCmd(char* buf, int buf_size) override;
private:
	double passive_speed_;
	double speed_max_;

	ControlArmRealtime* robot_rt_;
	
    GeneratePath generate_path_;
	std::vector<ur_data_type::Joint> joint_path_;

	ur_data_type::CartPose pose_curr_;
	ur_data_type::Joint joint_curr_;

	int repeat_times_;
	std::string cart_path_file_;
	std::string joint_path_file_;
	std::vector<ur_data_type::Joint> joint_path_file_running_ = {};

	pthread_t thread_;
	bool is_recurring_;

	int sleep_count_;
	
	int wait_loop_count_;
	int wait_loop_times_;

	bool moveToEndPose();
	bool moveToEndJoint(ur_data_type::Joint actual_joint);
	bool generateJointPathFromCartFile();
	bool generateJointPathFromJointFile();
	bool isReachedJointPosition(ur_data_type::Joint actual_joint, ur_data_type::Joint desire_joint);
	bool isReachedCartPosition(ur_data_type::CartesianPoint position);

	bool moveFromEndToStart();
	bool moveFromStartToEnd();

	void updateWaitLoopCount(int speed_level);
};

#endif
