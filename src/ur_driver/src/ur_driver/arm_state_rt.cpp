#include <cstring>
#include <mutex>
#include "arm_state_rt.h"

using namespace ur_data_type;

pthread_mutex_t actual_joint_lock_;

ArmStateRealtime::ArmStateRealtime()
{
	protocol_version_ = 0.0;                       
	time_ = 0.0;
    memset(&joint_position_, 0, sizeof(joint_position_));
    memset(&joint_vel_, 0, sizeof(joint_vel_));
    memset(&joint_acc_, 0, sizeof(joint_acc_));
    memset(&joint_current_, 0, sizeof(joint_current_));
    memset(&joint_torque_, 0, sizeof(joint_torque_));
               
    memset(&actual_joint_position_, 0, sizeof(actual_joint_position_));
    memset(&actual_joint_vel_, 0, sizeof(actual_joint_vel_));
    memset(&actual_current_, 0, sizeof(actual_current_));
    memset(&control_current_, 0, sizeof(control_current_));

    memset(&actual_tool_pose_, 0, sizeof(actual_tool_pose_));
    memset(&actual_tcp_vel_, 0, sizeof(actual_tcp_vel_));
    memset(&tcp_force_, 0, sizeof(tcp_force_));
    memset(&target_tool_pose_, 0, sizeof(target_tool_pose_));
    memset(&target_tcp_vel_, 0, sizeof(target_tcp_vel_));  
    memset(&tool_point_acc_, 0, sizeof(tool_point_acc_));    

    memset(&joint_control_modes_, 0, sizeof(joint_control_modes_));   
    memset(&elbow_point_, 0, sizeof(elbow_point_));   
    memset(&elbow_vel_, 0, sizeof(elbow_vel_));   

    memset(&joint_voltage_, 0, sizeof(joint_voltage_));  
    memset(&motor_temperatures_, 0, sizeof(motor_temperatures_));  

	digital_input_ = 0;                        
    digital_outputs_ = 0;                      

	robot_mode_ = 0.0;                        
	safety_mode_ = 0.0;        
    program_state_ = 0.0;

	controller_timer_ = 0.0;                     
	vel_scaling_ = 0.0; 
	linear_momentum_norm_ = 0.0;  

	main_voltage_ = 0.0;
	robot_voltage_ = 0.0;
	robot_current_ = 0.0;

    //actual_joint_lock_ = PTHREAD_MUTEX_INITIALIZER;
}

ArmStateRealtime::~ArmStateRealtime()
{
    pthread_mutex_destroy(&actual_joint_lock_);
}



double ArmStateRealtime::getProtocolVersion() const
{
    return protocol_version_;
}

void ArmStateRealtime::setProtocolVersion(double version)
{
    protocol_version_ = version;
}

double ArmStateRealtime::getTime()
{
    return time_;
}

ur_data_type::Joint ArmStateRealtime::getTargetJoint() const
{
    return joint_position_;
}

ur_data_type::Joint ArmStateRealtime::getTargetJointVel() const
{
    return joint_vel_;
}

ur_data_type::Joint ArmStateRealtime::getTargetJointAcc() const
{
    return joint_acc_;
}   

ur_data_type::Joint ArmStateRealtime::getTargetCurrent() const
{
    return joint_current_;
}

ur_data_type::Joint ArmStateRealtime::getTargetTorque() const
{
    return joint_torque_;
}

ur_data_type::Joint ArmStateRealtime::getActualJoint() 
{
    //const std::lock_guard<std::mutex> lock(actual_joint_lock_);
    pthread_mutex_lock(&actual_joint_lock_);
    ur_data_type::Joint joint = actual_joint_position_;
    pthread_mutex_unlock(&actual_joint_lock_);
    return joint;
}

ur_data_type::Joint ArmStateRealtime::getActualJointVel() const
{
    return actual_joint_vel_;
}

ur_data_type::Joint ArmStateRealtime::getActualJointCurrent() const
{ 
    return actual_current_;
}

ur_data_type::Joint ArmStateRealtime::getControlJointCurrent() const
{
    return control_current_;
}

ur_data_type::Joint ArmStateRealtime::getActualJointVoltage() const
{
    return joint_voltage_;
}

ur_data_type::CartPose ArmStateRealtime::getActualToolPose() const
{
    return actual_tool_pose_;
}

ur_data_type::CartPose ArmStateRealtime::getActualTcpVel() const
{
    return actual_tcp_vel_;
}

ur_data_type::CartPose ArmStateRealtime::getTcpForce() const
{
    return tcp_force_;
}

ur_data_type::CartPose ArmStateRealtime::getTargetToolPose() const
{
    return target_tool_pose_;
}

ur_data_type::CartPose ArmStateRealtime::getTargetTcpVel() const
{
    return target_tcp_vel_;
}

ur_data_type::CartesianPoint ArmStateRealtime::getToolPointAcc() const
{
    return tool_point_acc_;
} 

ur_data_type::CartesianPoint ArmStateRealtime::getElbowPoint() const
{
    return elbow_point_;
}

ur_data_type::CartesianPoint ArmStateRealtime::getElbowVel() const
{
    return elbow_vel_;
}

double ArmStateRealtime::getLinearMomentumNorm() const
{
    return linear_momentum_norm_;
}

double ArmStateRealtime::getSpeedScaling() const
{
    return vel_scaling_;
}

uint64_t ArmStateRealtime::getDigitalInputs() const
{
    return digital_input_;
}

uint64_t ArmStateRealtime::getDigitalOutputs() const
{
    return digital_outputs_;
}

uint64_t ArmStateRealtime::getRobotMode() const
{
    return robot_mode_;
}

ur_data_type::Joint ArmStateRealtime::getAllJointMode() const
{
    return joint_control_modes_;
}
uint64_t ArmStateRealtime::getSafetyMode() const
{
    return safety_mode_;
}

uint64_t ArmStateRealtime::getProgramState() const
{
    return program_state_;
}

double ArmStateRealtime::getControllerTimer() const
{
    return controller_timer_;
}

ur_data_type::Joint ArmStateRealtime::getMotorTemperatures() const
{
    return motor_temperatures_;
}

double ArmStateRealtime::getMainVoltage() const
{
    return main_voltage_;
}

double ArmStateRealtime::getRobotVoltage() const
{
    return robot_voltage_;
}

double ArmStateRealtime::getRobotCurrent() const
{
    return robot_current_;
}
