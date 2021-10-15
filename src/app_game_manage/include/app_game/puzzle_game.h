
#ifndef PUZZLE_GAME_H
#define PUZZLE_GAME_H

#include <future>
#include "force_datatype.h"
#include "ft_sensor.h"
#include "app_game.h"
#include "force_control.h"
#include "ur_kinematics.h"
#include "solve_jacobian.h"
#include "matrix_inverse.h"
#include "planar_datatype.h"
#include "puzzle_game_msg.pb.h"

#define PUZZLE_LOOP_NUM (10)
#define PUZZLE_PATH_WIDTH (0.005)
#define TEST_COUNT (8)
#define RANDOM_TIME (500.0)

class PuzzleGame : public AppGame
{
public:
    PuzzleGame();
    ~PuzzleGame();
   	virtual bool initParams(app_game_model::AppGameParams params) override;
	virtual bool start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor) override;
	virtual void pause() override;
	virtual void stop() override;
	virtual void replay() override;
    virtual void process() override;
    virtual std::string getName() override;
    bool isRunning() { return is_processing_; }
    virtual void encodeRuntimeInfo(char* buf, int &buf_size) override;
    virtual bool decodeRuntimeCmd(char* buf, int buf_size) override;

private:
    unsigned int game_action_;
    unsigned int game_order_;  
    int line_num_;
    int counter_;
    int cut_distance_;
    int lose_num_;
    int score_;
    int fix_times_; //flag_fix
    int tcp_counter_;

    bool is_cutting_;
    double cut_total_distance_;
    double puzzle_rigidity_;

    int tcp_state_;
    double speed_;
    int speed_level_;
    double max_force_;
    int fixing_state_;
    double start_time_;

	double puzzle_path_wide_;

    planar_data_type::PlanarPoint last_corner_point_;

    int cutting_start_index_;
    int ordinal_cutting_start_index_;
    int inverse_cutting_start_index_;  

    double move_force_;
    double cut_force_;
    double random_force_max_;
    double random_during_;

    float puzzle_cut_switch_;
    float puzzle_cut_slow_;
    double force_down_;
    double force_up_;

    double transition_wp2base_[4][4];
    ur_data_type::CartPose tool_pose_in_base_;
    ur_data_type::CartPose tool_pose_in_wp_;

    ControlArmRealtime *control_arm_rt_;
    FTSensor* ft_sensor_;
    UrKinematics ur_kine_;
    ForceControl force_control_;
    SloveJacobian solve_jacobian_;
    MatrixInverse44 matrix_inverse_44_;

    planar_data_type::PlanarForceValue pazzle_fault_force_;
    planar_data_type::PlanarForceValue passive_force_;

    PuzzleGameMsg::Unity2Ubuntu unity2ubuntu_;
    PuzzleGameMsg::Ubuntu2Unity ubuntu2unity_;

	ur_data_type::Joint cmd_speed_;
	double cmd_acc_;
    double height_;

    void initRuntimeParams();

    void initCutAndMoveForce(unsigned int force_level);
    void forceCalculate();
    double cadlculateTotalDistance(planar_data_type::PlanarPoint point_last, planar_data_type::PlanarPoint point_actual);
 
    void getTcpPoseInWpCoordinate(
        ur_data_type::CartPose tool_pose_in_base,
        double trans_workpiece2base[4][4],
        ur_data_type::CartPose &tcp_pose_in_wp);
    bool onceProcess();

    void processPassiveForceOnMoving(ur_data_type::CartPose tcp_pose_in_base);
    void processPassiveForceOnCuting(ur_data_type::CartPose tcp_pose_in_base);
    void processPassiveForceOnAparting(ur_data_type::CartPose tcp_pose_in_base, int puzzle_rigidity, double path_wide);
    void processPassiveForceOnFixing(ur_data_type::CartPose tcp_pose_in_base, int puzzle_rigidity, double path_wide);

    void updatePassiveForceOnProcessing(ur_data_type::CartPose tcp_pose_in_base);
    void updatePassiveForceOnReplaying(ur_data_type::CartPose tcp_pose_in_base);
    void updatePassiveForceOnWaiting();
    void updatePassiveForceOnStoping();
    void updatePassiveForceByTrainMode(
        int train_mode, 
        double puzzle_rigidity,
        double path_wide,
        planar_data_type::PlanarPoint start,
        planar_data_type::PlanarPoint end,
        planar_data_type::PlanarPoint tcp
        );
    void updatePassiveForceByTrainMode(
        int train_mode, 
        planar_data_type::PlanarPoint start,
        planar_data_type::PlanarPoint end,
        planar_data_type::PlanarPoint tcp);
    void calculatePassiveForceByTrainMode(int train_mode);
    void calculateTotalDistance(planar_data_type::PlanarPoint point_last, 
        planar_data_type::PlanarPoint point_actual,
        double &total_distance);
    float calculateScore(double start_time, int& lose_number, double& cut_length, double& cut_cntegral);

    double getWallTime();
    void packageCommData();
};

#endif
