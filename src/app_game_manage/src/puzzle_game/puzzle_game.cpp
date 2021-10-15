#include <cstring>
#include <sys/time.h>
#include <unistd.h>

#include "path_file.h"
#include "thread_pool.h"
#include "puzzle_game.h"


using namespace ur_data_type;
using namespace app_game_model;
using namespace force_data_type;
using namespace math_calculate;
using namespace PuzzleGameMsg;

PuzzleGame::PuzzleGame()
{
    is_processing_ = false;
    running_state_ = IDEL;
    training_mode_ = NONE;
    game_action_ = PUZZEL_IDEL;
    line_num_ = 0;
    game_order_= ORDINAL;

    cutting_start_index_ = 0;
    ordinal_cutting_start_index_ = 0;
    inverse_cutting_start_index_ = 0;

    counter_ = 0;
    cut_distance_ = 0;
    lose_num_= 0;
    score_= 0;
    fix_times_ = 0;
    tcp_counter_ = 0;

    is_cutting_ = false;
    cut_total_distance_ = 0.0;
    puzzle_rigidity_ = 2;

    tcp_state_ = TCP_HANG;
    speed_ = 0;
    speed_level_ = 0;
    max_force_ = 0.0;
    fixing_state_ = EXIT_FIXING;
    start_time_ = 0.0;

    move_force_ = 0.0;
    cut_force_ = 0.0;
    random_force_max_ = 0.01;
    random_during_= 500.0;

    puzzle_cut_switch_ = 0.009;
    puzzle_cut_slow_ = 0.02;
    force_down_ = 5.0f;
    force_up_ = -1.0f;
    puzzle_path_wide_ = 0.005f;

    last_corner_point_.x = 0;
    last_corner_point_.y = 0;

    passive_force_.fx = 0.0;
    passive_force_.fy = 0.0;
    pazzle_fault_force_.fx = 0.0;
    pazzle_fault_force_.fy = 0.0;

    memset(&tool_pose_in_base_, 0, sizeof(tool_pose_in_base_));
    memset(&tool_pose_in_wp_, 0, sizeof(tool_pose_in_wp_));
    memset(&transition_wp2base_, 0, sizeof(transition_wp2base_)); // init

    control_arm_rt_ = nullptr;

    ubuntu2unity_.add_tcp_pose(0);
    ubuntu2unity_.add_tcp_pose(0); 
    ubuntu2unity_.add_tcp_pose(0);   

    height_ = 0.0;
    cmd_acc_ = 0.0;
    memset(&cmd_speed_, 0, sizeof(cmd_speed_));
    
    is_processing_ = false;
}

PuzzleGame::~PuzzleGame()
{

}

bool PuzzleGame::initParams(app_game_model::AppGameParams params)
{
    initCutAndMoveForce(params.force_level);

    ur_kine_.calibrateWorkpiece(transition_wp2base_);

    params_ = params;

    params_.sensor_info.is_sensor_used = params.sensor_info.is_sensor_used;
    if (params_.sensor_info.is_sensor_used)
    {
        params_.sensor_info.remove_zero_bias = params.sensor_info.remove_zero_bias;
        params_.sensor_info.sensor_ip = params.sensor_info.sensor_ip;
    }

    return true;
}

bool PuzzleGame::start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor)
{
    if (params_.force_level == FORCE_LEVEL_ZERO)
    {
        printf("PuzzleGame: start failed, force level: %d\n", params_.force_level);
        return false; 
    }

    if (control_arm_rt == nullptr)
    {
        printf("PuzzleGame: start failed, control_arm_rt pointer is nullptr\n");
        return false;
    }
    control_arm_rt_ = control_arm_rt;
    usleep(1000000);

    Joint actual_joint = control_arm_rt_->getActualJoint();
    if(actual_joint.jVal[0] < DOUBLE_ZERO
        && actual_joint.jVal[1] < DOUBLE_ZERO
        && actual_joint.jVal[2] < DOUBLE_ZERO
        && actual_joint.jVal[3] < DOUBLE_ZERO
        && actual_joint.jVal[4] < DOUBLE_ZERO
        && actual_joint.jVal[5] < DOUBLE_ZERO
        )
    {
        printf("PuzzleGame: start failed, control_arm_rt point error.\n");
        return false; 
    }

    /* start ft sensor*/
    ft_sensor_ = ft_sensor;
    FTSensorData sensor_data = ft_sensor_->getSensorData();
    printf("Warnning: ManualMovement: onceProcess: sensor_data %lf, %lf, %lf, %lf, %lf, %lf\n",
        sensor_data.force.fx, sensor_data.force.fy, sensor_data.force.fz,
        sensor_data.torque.tx, sensor_data.torque.ty, sensor_data.torque.tz);

    if (fabs(sensor_data.force.fx) < DOUBLE_ZERO
        && fabs(sensor_data.force.fy) < DOUBLE_ZERO
        && fabs(sensor_data.force.fz) < DOUBLE_ZERO
        && fabs(sensor_data.torque.tx) < DOUBLE_ZERO
        && fabs(sensor_data.torque.ty) < DOUBLE_ZERO
        && fabs(sensor_data.torque.tz) < DOUBLE_ZERO)
    {
        printf("ManualMovement: start ft sensor failed: data error.\n");
        return false;
    }

	CartPose actual_pose;
    double rotation[3][3] = {};
	ur_kine_.FK(actual_joint, actual_pose, rotation);
    height_ = actual_pose.point.z;

    force_control_.calculateEndEffectorForceBias(rotation, sensor_data.force);
    force_control_.updatePidFactors(params_.force_level);

    game_action_ = MOVING;
    running_state_ = PROCESSING;

    start_time_ = getWallTime();
    is_processing_ = true;

    ThreadPool::getInstance().registerHandler(this, THREAD_INDEX);
    return true;
}

void PuzzleGame::pause()
{
	if (running_state_ == WAITING)
	{
		running_state_ = PROCESSING;
	}
}

void PuzzleGame::stop()
{
    if (!is_processing_) return;

    is_processing_ = false;
    running_state_ = GameRunningState::STOPING;
	ThreadPool::getInstance().deregisterHandler(THREAD_INDEX);

    force_control_.initPidFactor();
}

void PuzzleGame::replay() 
{
    running_state_ = REPLAYING;
}

void PuzzleGame::process()
{
    if (!is_processing_) return;
    if (!control_arm_rt_->isConnected())
    {
        printf("ManualMovement: exited, because of ControlArmRT discnnected.\n");
        stop();
        return;
    }

    onceProcess();
}

bool PuzzleGame::onceProcess()
{
   Joint actual_joint = control_arm_rt_->getActualJoint();
    CartPose actual_tcp = control_arm_rt_->getActualToolPose();

    if (actual_joint.jVal[0] < 0.01
        && actual_joint.jVal[1] < 0.01
        && actual_joint.jVal[2] < 0.01
        && actual_joint.jVal[3] < 0.01
        && actual_joint.jVal[4] < 0.01
        && actual_joint.jVal[5] < 0.01
    )
    {
        printf("Warnning: ManualMovement: onceProcess: joint value error： %lf, %lf, %lf, %lf, %lf, %lf\n",
            actual_joint.jVal[0], actual_joint.jVal[1], actual_joint.jVal[2], 
            actual_joint.jVal[3], actual_joint.jVal[4], actual_joint.jVal[5]);
        return false;
    }

    FTSensorData sensor_data = ft_sensor_->getSensorData();

    if (force_control_.isForceBeyondLimit(ft_sensor_->getSensorData().force))
    {
        printf("Warnning: ManualMovement: sensor force is beyond limit, stoping.\n");
        return false;
    }

	CartPose actual_pose;
    double rotation[3][3] = {};
	ur_kine_.FK(actual_joint, actual_pose, rotation);

    force_data_type::ForceValue force_pid_output;
    force_data_type::TorqueValue torque_pid_output;
    if (!force_control_.calculatePidOutput(
            control_arm_rt_->getActualToolPose(), 
            control_arm_rt_->getActualJoint(),
            rotation,
            ft_sensor_->getSensorData(),
            force_pid_output,
            torque_pid_output
            )
        )
    {
        return false;
    }

    force_data_type::FTSensorData vw;

    switch (params_.apply_force_mode)
    {
        case Compliance:
        {
            forceCalculate();
            if (!force_control_.calculateComplianceVW(force_pid_output, torque_pid_output, vw))
            {
                return false;
            }
        }
        break;
        case Plane:
        {
            forceCalculate();
            force_control_.calculatePlaneVW(control_arm_rt_->getActualJoint(), 
                force_pid_output, torque_pid_output, height_, vw);
        }
        break;
        default:;
    };

    Joint joint_speed;
    if(!solve_jacobian_.inverseJacobian(control_arm_rt_->getActualJoint(), vw, joint_speed))
    {
        printf("Warnning: ManualMovement: inverseJacobian failed.\n");
        return false;
    }

    if(!force_control_.oddProtection(control_arm_rt_->getActualJoint(), joint_speed))
    {
        printf("Warnning: ManualMovement:oddProtection failed.\n");
        return false;
    }

    generateCmdJointSpeed(control_arm_rt_->getActualJointVel(), joint_speed, vw, params_.sensor_info.remove_zero_bias, cmd_speed_, cmd_acc_);
    if(!control_arm_rt_->speedJointMove(cmd_speed_, cmd_acc_))
    {
        printf("Warnning: ManualMovement: speedJointMove failed.\n");
        return false;
    }
    else
    {
        printf("ManualMovement: speedJointMove successful.\n");
    }
    return true;
}

void PuzzleGame::forceCalculate()
{
    if (unity2ubuntu_.ByteSize() == 0)
    {
        printf("Warning: PuzzleGame: unity2ubuntu is empty.\n");
        return;
    }

    tool_pose_in_base_ = control_arm_rt_->getActualToolPose();
    getTcpPoseInWpCoordinate(tool_pose_in_base_, transition_wp2base_, tool_pose_in_wp_);

    switch (running_state_)
    {
        case PROCESSING:
        {
            updatePassiveForceOnProcessing(tool_pose_in_base_);
        }
            break;
        case WAITING:
        {
            updatePassiveForceOnWaiting();
        }
            break;
        case STOPING:
        {
            updatePassiveForceOnStoping();
        }
            break;
        case REPLAYING:
        {
            updatePassiveForceOnReplaying(tool_pose_in_base_);
        }
            break;
        default:;
    };

    force_control_.updateForceInToolFrame(control_arm_rt_->getActualJoint());
    packageCommData();
}

void PuzzleGame::updatePassiveForceOnProcessing(ur_data_type::CartPose tcp_pose_in_base)
{
    switch (game_action_)
    {
        case MOVING:
        {
             processPassiveForceOnMoving(tcp_pose_in_base);
        }
        break;
        case CUTTING:
        {
            processPassiveForceOnCuting(tcp_pose_in_base);
        }
        break;
        case APARTING:
        {
            processPassiveForceOnAparting(tcp_pose_in_base, puzzle_rigidity_, puzzle_path_wide_);
        }
        break;
        case FIXTING:
        {
            processPassiveForceOnFixing(tcp_pose_in_base, puzzle_rigidity_, puzzle_path_wide_);
        }
        break;
        default:;
    };
}

void PuzzleGame::updatePassiveForceOnWaiting()
{
    ubuntu2unity_.set_game_replay_state(PAUSE);
    passive_force_.fx = 0.0;
    passive_force_.fy = 0.0;
    force_control_.setPlanarPassiveForce(passive_force_);
    force_control_.initPidOutput();
}

void PuzzleGame::updatePassiveForceOnReplaying(ur_data_type::CartPose tcp_pose_in_base)
{
    passive_force_.fx = 0.0;
    passive_force_.fy = 0.0;
    force_control_.setPlanarPassiveForce(passive_force_);

    line_num_ = 0;
    counter_ = 0;
    cut_distance_ = 0;
    lose_num_ = 0;

    initRuntimeParams();

    pazzle_fault_force_.fx = 0.0;
    pazzle_fault_force_.fy = 0.0;

    last_corner_point_.x = tcp_pose_in_base.point.x;
    last_corner_point_.y = tcp_pose_in_base.point.y;

    static int loop_times = 0;

    ubuntu2unity_.set_tcp_state(TCP_HANG);
    ubuntu2unity_.set_game_result(GAMING);
    if (loop_times >= PUZZLE_LOOP_NUM)
    {
        loop_times = 0;
        running_state_ = PROCESSING;
    }
    else
    {
       loop_times++;
       ubuntu2unity_.set_game_replay_state(REPLAY);
    }

    game_action_ = PuzzleGameAction::MOVING;
}

void PuzzleGame::updatePassiveForceOnStoping()
{
    passive_force_.fx = 0.0;
    passive_force_.fy = 0.0;
    force_control_.setPlanarPassiveForce(passive_force_);

    pazzle_fault_force_.fx = 0.0;
    pazzle_fault_force_.fy = 0.0;

    line_num_ = 0;
    counter_ = 0;
    cut_distance_ = 0;
    lose_num_ = 0;

    initRuntimeParams();
    game_action_ = PuzzleGameAction::MOVING;
    ubuntu2unity_.set_tcp_state(TCP_HANG);
    ubuntu2unity_.set_game_result(GAMING);
    ubuntu2unity_.set_game_replay_state(BACK);

    game_action_ = PuzzleGameAction::MOVING;
}

void PuzzleGame::encodeRuntimeInfo(char* buf, int &buf_size)
{
    buf_size = unity2ubuntu_.ByteSize();
    ubuntu2unity_.SerializeToArray(buf, buf_size);
}

bool PuzzleGame::decodeRuntimeCmd(char* buf, int buf_size)
{
   if(!unity2ubuntu_.ParseFromArray(buf, buf_size))
    {
        printf("PuzzleGame: parse data failed.\n");
        return false;
    }

    return true;
}

void PuzzleGame::packageCommData()
{
    ubuntu2unity_.set_game_length(cut_distance_);
    ubuntu2unity_.set_tcp_pose(0, tool_pose_in_base_.point.x);
    ubuntu2unity_.set_tcp_pose(1, tool_pose_in_base_.point.y);
    ubuntu2unity_.set_tcp_pose(2, tool_pose_in_base_.rpy.rz);
    ubuntu2unity_.set_sensor_force_z(ft_sensor_->getSensorData().force.fz);
    ubuntu2unity_.set_game_device(SCREEN);
#if 0
    std::async(std::launch::async, this()
	{
        ubuntu2unity_.set_game_length(cut_distance_);
        ubuntu2unity_.set_tcp_pose(0, tool_pose_in_base_.point.x);
        ubuntu2unity_.set_tcp_pose(1, tool_pose_in_base_.point.y);
        ubuntu2unity_.set_tcp_pose(2, tool_pose_in_base_.rpy.rz);
        ubuntu2unity_.set_sensor_force_z(ft_sensor_->getSensorData().force.fz);
        ubuntu2unity_.set_game_device(SCREEN);
	});
#endif 
}

std::string PuzzleGame::getName() 
{
    return "TrajRecurrent";
}
