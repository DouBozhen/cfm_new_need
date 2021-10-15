#include <cstring>
#include <arpa/inet.h>
#include "arm_state.h"

void ArmState::decodeJointdataFromRobotStateMsg(uint8_t* buf, unsigned int offset, JointData &joint_data)
{
	for (int i = 0; i < 6; ++i)
	{
		uint64_t temp = *(uint64_t*)&buf[offset];
		joint_data.actual_joint.jVal[i] = ntohd(temp);
        offset += sizeof(temp);

    	temp = *(uint64_t*)&buf[offset];
		joint_data.target_joint.jVal[i] = ntohd(temp);
		offset += sizeof(temp);

		temp = *(uint64_t*)&buf[offset];
		joint_data.actual_joint_vel.jVal[i] = ntohd(temp);
		offset += sizeof(temp);

		uint32_t temp1 = *(uint32_t*)&buf[offset];
		joint_data.actual_current[i] = ntohf(temp1);
		offset += sizeof(temp1);
        
		temp1 = *(uint32_t*)&buf[offset];
		joint_data.actual_voltage[i] = ntohf(temp1);
		offset += sizeof(temp1);

		temp1 = *(uint32_t*)&buf[offset];
		joint_data.t_motor[i] = ntohf(temp1);
		offset += sizeof(temp1);

		temp1 = *(uint32_t*)&buf[offset];
		joint_data.t_micro[i] = ntohf(temp1);
		offset += sizeof(temp1);

		joint_data.joint_mode[i] = *(uint8_t*)&buf[offset] ;
		offset += sizeof(joint_data.joint_mode[i]);
	}
}

void ArmState::decodeRobotModeFromRobotStateMsg(uint8_t* buf, unsigned int offset, RobotModeData &robot_mode)
{
 	robot_mode.time_stamp = *(uint64_t*)&buf[offset];
	offset += sizeof(robot_mode.time_stamp);

	uint8_t tmp = buf[offset]; 
	if (tmp > 0) robot_mode.is_connected = true;
	else robot_mode.is_connected = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) robot_mode.is_enabled = true;
	else robot_mode.is_enabled = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) robot_mode.is_powered_on = true;
	else robot_mode.is_powered_on = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) robot_mode.is_emergency_stopped = true;
	else robot_mode.is_emergency_stopped = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) robot_mode.is_Protective_stopped = true;
	else robot_mode.is_Protective_stopped = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) robot_mode.is_program_running = true;
	else robot_mode.is_program_running = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) robot_mode.is_program_paused = true;
	else robot_mode.is_program_paused = false;
	offset += sizeof(tmp);

	robot_mode.robot_mode = buf[offset];
	offset += sizeof(robot_mode.robot_mode);

	uint64_t temp;
	if (getProtocolVersion() > 2.)
	{
		robot_mode.control_mode = buf[offset];
		offset += sizeof(robot_mode.control_mode);

		temp = buf[offset];
		robot_mode.target_speed_fraction = ntohd(temp);   //double
        offset += sizeof(temp);
	}
	temp = *(uint64_t *)&buf[offset];
	offset += sizeof(temp);
	robot_mode.speed_scaling = ntohd(temp);      //double

	temp = *(uint64_t *)&buf[offset];
	offset += sizeof(temp);
	robot_mode.target_speed_fraction_limit = ntohd(temp);
}

void ArmState::decodeCartInfoFromRobotStateMsg(uint8_t* buf, unsigned int offset, CartesianInfo &cart_info)
{
    uint64_t temp = *(uint64_t *)&buf[offset];
	cart_info.tool_pose.point.x = ntohd(temp);
    offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tool_pose.point.y = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tool_pose.point.z = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tool_pose.rpy.rx = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tool_pose.rpy.ry = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tool_pose.rpy.rz = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tcp_offset.point.x = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tcp_offset.point.y = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tcp_offset.point.z = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tcp_offset.rpy.rx = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tcp_offset.rpy.ry = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	cart_info.tcp_offset.rpy.rz = ntohd(temp);
	offset += sizeof(temp);
}

void ArmState::decodeMasterboardFromRobotStateMsg(uint8_t* buf, unsigned int offset, MasterBoardData &mb_data)
{
	if (getProtocolVersion() < 3.0)
	{
		int16_t digital_inputs = *(int16_t*)&buf[offset];
		mb_data.digital_inputs = ntohs(digital_inputs);
		offset += sizeof(digital_inputs);

		int16_t digital_outputs = *(int16_t*)&buf[offset];
		mb_data.digital_outputs = ntohs(digital_outputs);
		offset += sizeof(digital_outputs);
	}
	else
	{
		mb_data.digital_inputs = *(int*)&buf[offset];
		mb_data.digital_inputs = ntohl(mb_data.digital_inputs);   //int
		offset += sizeof(mb_data.digital_inputs);

		mb_data.digital_outputs = *(int*)&buf[offset];
		mb_data.digital_outputs = ntohl(mb_data.digital_outputs);
		offset += sizeof(mb_data.digital_outputs);
	}

	mb_data.analog_input_range_0 = *(char*)&buf[offset];
	offset += sizeof(mb_data.analog_input_range_0);

	mb_data.analog_input_range_1 = *(char *)&buf[offset];
	offset += sizeof(mb_data.analog_input_range_1);

	uint64_t temp = *(uint64_t*)&buf[offset];
	mb_data.analog_input_0 = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t*)&buf[offset];
	mb_data.analog_input_1 = ntohd(temp);
	offset += sizeof(temp);

	mb_data.analog_output_domain_0 = buf[offset];
	offset += sizeof(mb_data.analog_output_domain_0);

	mb_data.analog_output_domain_1 = buf[offset];
	offset += sizeof(mb_data.analog_output_domain_1);

	temp = *(uint64_t *)&buf[offset];
	offset += sizeof(temp);
	mb_data.analog_output_0 = ntohd(temp);

	temp = *(uint64_t *)&buf[offset];
	offset += sizeof(temp);
	mb_data.analog_output_1 = ntohd(temp);

	uint32_t temp1 = *(int32_t*)&buf[offset];
	mb_data.mb_temperature = ntohf(temp1);
	offset += sizeof(temp1);

	temp1 = *(int32_t *)&buf[offset];
	mb_data.robot_voltage_48V = ntohf(temp1);
	offset += sizeof(temp1);

	temp1 = *(int32_t *)&buf[offset];
	mb_data.robot_current = ntohf(temp1);
	offset += sizeof(temp1);

	temp1 = *(int32_t *)&buf[offset];
	mb_data.master_io_current = ntohf(temp1);
	offset += sizeof(temp1);

	mb_data.safety_mode = buf[offset];
	offset += sizeof(mb_data.safety_mode);

	mb_data.in_reduced_mode = buf[offset];
	offset += sizeof(mb_data.in_reduced_mode);

	mb_data.is_euromap67_installed = buf[offset];
	offset += sizeof(mb_data.is_euromap67_installed);

	if (mb_data.is_euromap67_installed != 0)
	{
		mb_data.euromap_inputs = *(int*)&buf[offset];
		mb_data.euromap_inputs = ntohl(mb_data.euromap_inputs);
		offset += sizeof(mb_data.euromap_inputs);

		mb_data.euromap_outputs = buf[offset];
		mb_data.euromap_outputs = ntohl(mb_data.euromap_outputs);
		offset += sizeof(mb_data.euromap_outputs);

		if (getProtocolVersion() < 3.0)
		{
			int16_t euromap_voltage = *(int16_t*)&buf[offset];// , euromap_current;
			mb_data.euromap_voltage_24V = ntohs(euromap_voltage);
			offset += sizeof(euromap_voltage);

			int16_t euromap_current = *(int16_t*)&buf[offset];
			mb_data.euromap_current = ntohs(euromap_current);
			offset += sizeof(euromap_current);
		}
		else
		{
			temp1 = *(uint32_t *)&buf[offset];
            mb_data.euromap_voltage_24V = ntohf(temp1);
			offset += sizeof(temp1);

			temp1 = *(uint32_t *)&buf[offset];
			mb_data.euromap_current = ntohf(temp1);
			offset += sizeof(temp1);
		}
	}
    
	mb_data.op_mode_selector_input = buf[offset]; // to check...
	offset += sizeof(mb_data.op_mode_selector_input);

	mb_data.three_position_enabling_input = buf[offset]; // to check...
	offset += sizeof(mb_data.three_position_enabling_input);
}


void ArmState::decodeToolDataFromRobotStateMsg(uint8_t* buf, unsigned int offset, ToolData &tool_data)
{
    memcpy(&tool_data.analog_input_range_0, &buf[offset], sizeof(tool_data.analog_input_range_0));//char
	offset += sizeof(tool_data.analog_input_range_0);
	memcpy(&tool_data.analog_input_range_1, &buf[offset], sizeof(tool_data.analog_input_range_1));
	offset += sizeof(tool_data.analog_input_range_1);

	uint64_t temp;
	memcpy(&temp, &buf[offset], sizeof(temp));
	tool_data.analog_input_0 = ntohd(temp);
	offset += sizeof(temp);

	memcpy(&temp, &buf[offset], sizeof(temp));
	tool_data.analog_input_1 = ntohd(temp);
	offset += sizeof(temp);

	uint32_t temp1;
	memcpy(&temp1, &buf[offset], sizeof(temp1));
	tool_data.tool_voltage_48V = ntohf(temp1);
	offset += sizeof(temp1);

	memcpy(&tool_data.tool_output_voltage, &buf[offset], sizeof(tool_data.tool_output_voltage));//unchar
	offset += sizeof(tool_data.tool_output_voltage);

	memcpy(&temp1, &buf[offset], sizeof(temp1));
	tool_data.tool_current = ntohf(temp1);
	offset += sizeof(temp1);

	memcpy(&temp1, &buf[offset], sizeof(temp1));
	tool_data.tool_temperature = ntohf(temp1);
	offset += sizeof(temp1);

	memcpy(&tool_data.tool_mode, &buf[offset], sizeof(tool_data.tool_mode));
	offset += sizeof(tool_data.tool_mode);
}

void ArmState::decodeKineInfoFromRobotStateMsg(uint8_t* buf, unsigned int offset, KinematicsInfo &kine_info)
{
	uint64_t temp;
	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
        kine_info.check_sum[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
        kine_info.dh_param.theta.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		kine_info.dh_param.a.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}
	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		kine_info.dh_param.d.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		kine_info.dh_param.alpha.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	uint32_t temp_data1;
	memcpy(&temp_data1, &buf[offset], sizeof(temp_data1));
	kine_info.calibration_status = ntohl(temp_data1);
	offset += sizeof(temp_data1);
}

void ArmState::decodeConfigDataFromRobotStateMsg(uint8_t* buf, unsigned int offset, ConfigurationData &config_data)
{
    uint64_t temp;
	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		config_data.joint_limit_min.jVal[i] = ntohd(temp);
		offset += sizeof(temp);

		memcpy(&temp, &buf[offset], sizeof(temp));
		config_data.joint_limit_max.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		config_data.joint_vel_max.jVal[i] = ntohd(temp);
		offset += sizeof(temp);

		memcpy(&temp, &buf[offset], sizeof(temp));
		config_data.joint_acc_max.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	memcpy(&temp, &buf[offset], sizeof(temp));
	config_data.joint_v_default = ntohd(temp);
	offset += sizeof(temp);

	memcpy(&temp, &buf[offset], sizeof(temp));
	config_data.joint_a_default = ntohd(temp);
	offset += sizeof(temp);

	memcpy(&temp, &buf[offset], sizeof(temp));
	config_data.tool_v_default = ntohd(temp);
	offset += sizeof(temp);

	memcpy(&temp, &buf[offset], sizeof(temp));
	config_data.tool_a_default = ntohd(temp);
	offset += sizeof(temp);

	memcpy(&temp, &buf[offset], sizeof(temp));
	config_data.eq_rad = ntohd(temp);
	offset += sizeof(temp);

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		config_data.dh_param.a.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
        config_data.dh_param.d.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
        config_data.dh_param.alpha.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	for (int i = 0; i < 6; ++i)
	{
		memcpy(&temp, &buf[offset], sizeof(temp));
		config_data.dh_param.theta.jVal[i] = ntohd(temp);
		offset += sizeof(temp);
	}

	uint32_t temp_data1;
	memcpy(&temp_data1, &buf[offset], sizeof(temp_data1));
	config_data.mb_version = ntohl(temp_data1);
	offset += sizeof(temp_data1);

	memcpy(&temp_data1, &buf[offset], sizeof(temp_data1));
	config_data.controller_box_type = ntohl(temp_data1);
	offset += sizeof(temp_data1);

	memcpy(&temp_data1, &buf[offset], sizeof(temp_data1));
	config_data.robot_type = ntohl(temp_data1);
	offset += sizeof(temp_data1);

	memcpy(&temp_data1, &buf[offset], sizeof(temp_data1));
	config_data.robot_subtype = ntohl(temp_data1);
	offset += sizeof(temp_data1);
}

void ArmState::decodeForceModeFromRobotStateMsg(uint8_t* buf, unsigned int offset, ForceModeData &force_mode)
{
	uint64_t temp;
	temp = *(uint64_t *)&buf[offset];
    force_mode.force.point.x = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	force_mode.force.point.y = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	force_mode.force.point.z = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	force_mode.force.rpy.rx = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	force_mode.force.rpy.ry = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	force_mode.force.rpy.rz = ntohd(temp);
	offset += sizeof(temp);

	temp = *(uint64_t *)&buf[offset];
	force_mode.dexterity = ntohd(temp);
	offset += sizeof(temp);
}

void ArmState::decodeAdditionalInfoFromRobotState(uint8_t* buf, unsigned int offset, AdditionalInfo &additional_info)
{
	uint8_t tmp = buf[offset];
	if (tmp > 0) additional_info.freedrive_button_pressed = true;
	else additional_info.freedrive_button_pressed = false;
	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) additional_info.freedrive_button_enabled = true;
	else additional_info.freedrive_button_enabled = false;
  	offset += sizeof(tmp);

	tmp = buf[offset];
	if (tmp > 0) additional_info.freedrive_io_enabled = true;
	else additional_info.freedrive_io_enabled = false;
	offset += sizeof(tmp);
}


