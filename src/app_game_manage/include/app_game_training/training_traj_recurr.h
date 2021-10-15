#ifndef TRAINING_TRAJ_RECURR_H
#define TRAINING_TRAJ_RECURR_H

#include <pthread.h>
#include <fstream>
#include <string>

#include "handler.h"
#include "control_arm_rt.h"
#include "ft_sensor.h"

#include "app_game_model.h"
#include "force_datatype.h"

#include "force_control.h"
#include "ur_kinematics.h"
#include "solve_jacobian.h"
#include "generate_path.h"

#define THREAD_INDEX 0
#define WAIT_LOOP_COUNT (500)

class TrainingTrajRecurr : public Handler
{
public:
	TrainingTrajRecurr();
	~TrainingTrajRecurr();

	bool initParams(app_game_model::AppGameParams params);
	void setComponent(ControlArmRealtime* control_arm_rt);
	bool start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor);
	void stop();
	void pause();
	void replay();
	virtual void process() override;
	virtual std::string getName() override;

	bool moveProximalLimitToHome(double vel);
	bool moveDistalLimitToHome(double vel);

	int generateProximalPath();
	int generateDistalPath();
	ur_data_type::Joint getProximalLimitPosition();
	ur_data_type::Joint getDistalLimitPosition();
	ur_data_type::Joint getHomePosition();

	bool isRunning() { return is_recurring_;} 
	bool isReachedHomePosition();
	bool isReachedHomePosition(ur_data_type::Joint actual_joint, ur_data_type::Joint actual_joint_vel);
	bool isReachedProximalLimitPosition();
	bool isReachedDistalLimitPosition();

private:
    unsigned int training_mode_;
    unsigned int running_state_;
    app_game_model::AppGameParams params_;
    bool remove_zero_bias_;
    bool is_recurring_;
	bool training_end_;

	double passive_speed_;
	double speed_max_;
	int repeat_times_;
	pthread_t thread_;

	int sleep_count_;

	ControlArmRealtime* robot_rt_;

	ur_data_type::CartPose pose_curr_;
	ur_data_type::Joint joint_curr_;
	ur_data_type::Joint joint_proximal_limit_;
	ur_data_type::Joint joint_distal_limit_;
	ur_data_type::Joint joint_home_;

    GeneratePath generate_path_proximal_;
	GeneratePath generate_path_distal_;
	std::vector<ur_data_type::Joint> joint_path_proximal_;
	std::vector<ur_data_type::Joint> joint_path_distal_;
	std::vector<ur_data_type::Joint> joint_path_all_;

	std::string cart_path_file_proximal_;
	std::string joint_path_file_proximal_;
	std::string cart_path_file_distal_;
	std::string joint_path_file_distal_;

	int wait_loop_count_;
	int wait_loop_times_;
	void updateWaitLoopCount(int speed_level);

	void initGameParams(app_game_model::AppGameParams params);

	bool generatePathAll();
	bool generatePathProximal(const char* file);
	bool generatePathDistal(const char* file);

	bool moveFromHomeToProximalLimit();
	bool moveFromHomeToDistalLimit();
	bool moveFromActualtoHome();

	bool isReachedJointPosition(
			ur_data_type::Joint actual_joint, 
			ur_data_type::Joint desire_joint, 
			ur_data_type::Joint actual_joint_speed);

	bool moveFromDistalToProximal(); /* first */
	bool moveFromProximalToDistal(); /* last */

	bool trajFitting();
};

#endif
