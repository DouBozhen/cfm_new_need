#ifndef UR_COMM_MESSAGE_DATA_H
#define UR_COMM_MESSAGE_DATA_H

#include "ur_datatype.h"

struct JointData
{
	ur_data_type::Joint actual_joint;
	ur_data_type::Joint target_joint;
	ur_data_type::Joint actual_joint_vel;
	std::array<float, 6> actual_current;
	std::array<float, 6> actual_voltage;
	std::array<float, 6> t_motor;
	std::array<float, 6> t_micro;
	std::array<uint8_t, 6> joint_mode;
};

struct CartesianInfo
{
	ur_data_type::CartPose tool_pose;
	ur_data_type::CartPose tcp_offset;
};

struct MasterBoardData
{
	int digital_inputs;
	int digital_outputs;
	char analog_input_range_0;
	char analog_input_range_1;
	double analog_input_0;
	double analog_input_1;
	char analog_output_domain_0;
	char analog_output_domain_1;
	double analog_output_0;
	double analog_output_1;
	float mb_temperature;
	float robot_voltage_48V;
	float robot_current;
	float master_io_current;
	unsigned char safety_mode;
	unsigned char in_reduced_mode;
	char is_euromap67_installed;
	int euromap_inputs;
	int euromap_outputs;
	float euromap_voltage_24V;
	float euromap_current;
	uint8_t op_mode_selector_input;
	uint8_t three_position_enabling_input;
};

struct ToolData
{
	char analog_input_range_0;
	char analog_input_range_1;
	double analog_input_0;
	double analog_input_1;
	float tool_voltage_48V;
	unsigned char tool_output_voltage;
	float tool_current;
	float tool_temperature;
	uint8_t tool_mode;
};

struct KinematicsInfo
{
	std::array<uint32_t, 6>  check_sum;
	ur_data_type::DHParam dh_param;
	uint32_t calibration_status;
};

//add
struct ConfigurationData
{
	ur_data_type::Joint joint_limit_min;
	ur_data_type::Joint joint_limit_max;
	ur_data_type::Joint joint_vel_max;
	ur_data_type::Joint joint_acc_max;
	double joint_v_default;
	double joint_a_default;
	double tool_v_default;
	double tool_a_default;
	double eq_rad;
	ur_data_type::DHParam dh_param;
	int mb_version;
	int controller_box_type;
	int robot_type;
	int robot_subtype;
};

//add
struct ForceModeData
{
	ur_data_type::CartPose force;
	double dexterity;
};

//add
struct AdditionalInfo
{
	bool freedrive_button_pressed;
	bool freedrive_button_enabled;
	bool freedrive_io_enabled;
};

struct RobotModeData
{
	uint64_t time_stamp;
	bool is_connected;
	bool is_enabled;
	bool is_powered_on;
	bool is_emergency_stopped;
	bool is_Protective_stopped;
	bool is_program_running;
	bool is_program_paused;
	unsigned char robot_mode;
	unsigned char control_mode;
	double target_speed_fraction;
	double speed_scaling;
	double target_speed_fraction_limit;
};

#endif