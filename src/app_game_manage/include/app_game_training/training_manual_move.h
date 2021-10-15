#ifndef TRAINING_MANUAL_MOVE_H
#define TRAINING_MANUAL_MOVE_H

#include "record_path.h"
#include "manual_movement.h"

class TrainingManualMove : public Handler
{
public:
    TrainingManualMove();
	~TrainingManualMove();
   	bool initParams(app_game_model::AppGameParams params);
	bool start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor);
	void pause();
	void stop();
	void replay(); 
    virtual void process() override;
    virtual std::string getName() override;
    bool isRunning() { return is_moving_; }

private:
    unsigned int training_mode_;
    unsigned int running_state_;
    app_game_model::AppGameParams params_;
    bool is_moving_;

    ControlArmRealtime *control_arm_rt_;
	FTSensor* ft_sensor_;
	ManualMovement manual_move_;

	bool remove_zero_bias_;
	RecordPath record_path_;
	RecordLimit record_path_limit_;

	ur_data_type::Joint start_joint_position_;
	ur_data_type::Joint end_joint_position_;

	std::string cart_pose_file_;
	std::string cart_pose_file_impendance_;
	std::string joint_file_;
	std::string joint_file_impendance_;

	ur_data_type::Joint proximal_limit_position_;
	ur_data_type::Joint distal_limit_position_;
	ur_data_type::Joint home_position_;

    bool createManualMovement();
    void dropManualMovement();

	bool openRecordPath();
	void closeRecordPath();
};

#endif
