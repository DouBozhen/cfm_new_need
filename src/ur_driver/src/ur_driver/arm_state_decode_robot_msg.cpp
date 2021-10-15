#include <cstring>
#include <arpa/inet.h>
#include "arm_state.h"

void ArmState::decodeVersionFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, 
    VersionMessage &version_msg)
{
	version_msg.project_name_size = buf[offset];
	offset += sizeof(version_msg.project_name_size);

	memcpy(&version_msg.project_name, &buf[offset], sizeof(char)* version_msg.project_name_size); 
	offset += version_msg.project_name_size;
	version_msg.project_name[version_msg.project_name_size] = '\0';
	
    version_msg.major_version = buf[offset];
	offset += sizeof(version_msg.major_version);
	version_msg.minor_version = buf[offset];
	offset += sizeof(version_msg.minor_version);

	version_msg.svn_revision = *(int*)&buf[offset]; // bug_fix_version
	offset += sizeof(version_msg.svn_revision);
	version_msg.svn_revision = ntohl(version_msg.svn_revision);

	version_msg.build_number = *(int *)&buf[offset];
	offset += sizeof(version_msg.build_number);
	version_msg.build_number = ntohl(version_msg.build_number);

	memcpy(&version_msg.build_date, &buf[offset], sizeof(char) * (len - offset));   
	version_msg.build_date[len - offset] = '\0';
}

void ArmState::decodeSafetyModeFromRobotMsg(uint8_t* buf, 
    unsigned int offset, int32_t len, SafetyModeMessage &safety_mode_msg)
{
    safety_mode_msg.robot_msg_code = *(int*)&buf[offset];
	offset += sizeof(safety_mode_msg.robot_msg_code);
	safety_mode_msg.robot_msg_code = ntohl(safety_mode_msg.robot_msg_code);

	safety_mode_msg.robot_msg_arg = *(int*)&buf[offset];
	offset += sizeof(safety_mode_msg.robot_msg_arg);
	safety_mode_msg.robot_msg_arg = ntohl(safety_mode_msg.robot_msg_arg);

	if (getProtocolVersion() >= 3.7)
	{
		safety_mode_msg.safety_mode_type = buf[offset];
		offset += sizeof(safety_mode_msg.safety_mode_type);
		safety_mode_msg.safety_mode_type = ntohl(safety_mode_msg.safety_mode_type);

		safety_mode_msg.report_data_type = *(int32_t*)&buf[offset];
		offset += sizeof(safety_mode_msg.report_data_type);
		safety_mode_msg.report_data_type = ntohl(safety_mode_msg.report_data_type);

		safety_mode_msg.report_data = *(int32_t*)&buf[offset];
		offset += sizeof(safety_mode_msg.report_data);
		safety_mode_msg.report_data = ntohl(safety_mode_msg.report_data);
	}

	memcpy(&safety_mode_msg.text_msg, &buf[offset], sizeof(char) * len);   // useless
	safety_mode_msg.text_msg[len] = '\0';
}

void ArmState::decodeKeyFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, KeyMessage &key_msg)
{
	key_msg.key_msg_code = *(int*)&buf[offset];
	offset += sizeof(key_msg.key_msg_code);
	key_msg.key_msg_code = ntohl(key_msg.key_msg_code);

	key_msg.key_msg_arg = *(int*)&buf[offset];
	offset += sizeof(key_msg.key_msg_arg);
	key_msg.key_msg_arg = ntohl(key_msg.key_msg_arg);

	key_msg.key_title_size = buf[offset];
	offset += sizeof(key_msg.key_title_size);

	memcpy(&key_msg.key_msg_title, &buf[offset], sizeof(char) * key_msg.key_title_size); 
	offset += key_msg.key_title_size;
	key_msg.key_msg_title[key_msg.key_title_size] = '\0';

	memcpy(&key_msg.key_text_msg, &buf[offset], sizeof(char) * (len - offset)); 
	key_msg.key_text_msg[len - offset] = '\0';
}

void ArmState::decodeRobotCommFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, 
    RobotCommMessage &robot_comm_msg)
{
	robot_comm_msg.robot_msg_code = *(int*)&buf[offset];
	offset += sizeof(robot_comm_msg.robot_msg_code);
	robot_comm_msg.robot_msg_code = ntohl(robot_comm_msg.robot_msg_code);

	robot_comm_msg.robot_msg_arg = *(int*)&buf[offset];
	offset += sizeof(robot_comm_msg.robot_msg_arg);
	robot_comm_msg.robot_msg_arg = ntohl(robot_comm_msg.robot_msg_arg);

	robot_comm_msg.warning_level = *(int*)&buf[offset];
	offset += sizeof(robot_comm_msg.warning_level);
	robot_comm_msg.warning_level = ntohl(robot_comm_msg.warning_level);

	if (ArmState::getProtocolVersion() >= 3.7)
	{
		memcpy(&robot_comm_msg.robot_msg_data_type, &buf[offset], sizeof(robot_comm_msg.robot_msg_data_type));
		offset += sizeof(robot_comm_msg.robot_msg_data_type);
		robot_comm_msg.robot_msg_data_type = ntohl(robot_comm_msg.robot_msg_data_type);

		memcpy(&robot_comm_msg.robot_msg_data, &buf[offset], sizeof(robot_comm_msg.robot_msg_data));
		offset += sizeof(robot_comm_msg.robot_msg_data);
		robot_comm_msg.robot_msg_data = ntohl(robot_comm_msg.robot_msg_data);
    }

	memcpy(&robot_comm_msg.text_msg, &buf[offset], sizeof(char) * (len - offset));   
	robot_comm_msg.text_msg[len - offset] = '\0';
}

void ArmState::decodeLabelFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, LabelMessage &lable_msg)
{
	lable_msg.id = *(int*)&buf[offset];
	offset += sizeof(lable_msg.id);
	lable_msg.id = ntohl(lable_msg.id);

	memcpy(&lable_msg.label_text_msg, &buf[offset], sizeof(char) * (len - offset));   
	lable_msg.label_text_msg[len - offset] = '\0';
}

void ArmState::decodePopupFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, PopupMessage &popup_msg)
{
    popup_msg.request_id = *(int*)&buf[offset];
	offset += sizeof(popup_msg.request_id);
	popup_msg.request_id = ntohl(popup_msg.request_id);

	popup_msg.request_type = buf[offset];
	offset += sizeof(popup_msg.request_type);
	popup_msg.request_type = ntohl(popup_msg.request_type);

	uint8_t tmp;
	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0) popup_msg.warning = true;
	else popup_msg.warning = false;
	offset += sizeof(tmp);

	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0) popup_msg.error = true;
	else popup_msg.error = false;
	offset += sizeof(tmp);

	memcpy(&tmp, &buf[offset], sizeof(tmp));
	if (tmp > 0) popup_msg.blocking = true;
	else popup_msg.blocking = false;
	offset += sizeof(tmp);

	popup_msg.popup_msg_title_size = *(uint8_t*)&buf[offset];
	offset += sizeof(popup_msg.popup_msg_title_size);
	popup_msg.popup_msg_title_size = ntohl(popup_msg.popup_msg_title_size);

	memcpy(&popup_msg.popup_msg_title, &buf[offset], sizeof(char) * popup_msg.popup_msg_title_size); 
	offset += popup_msg.popup_msg_title_size;
	popup_msg.popup_msg_title[popup_msg.popup_msg_title_size] = '\0';

	memcpy(&popup_msg.popup_text_msg, &buf[offset], sizeof(char) * (len - offset));   
	popup_msg.popup_text_msg[len - offset] = '\0';
}

void ArmState::decodeRequestFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, RequestValueMessage &request_msg)
{
	request_msg.request_id = *(int*)&buf[offset];
	offset += sizeof(request_msg.request_id);
	request_msg.request_id = ntohl(request_msg.request_id);

	request_msg.request_type = buf[offset];
	offset += sizeof(request_msg.request_type);
	request_msg.request_type = ntohl(request_msg.request_type);

	memcpy(&request_msg.request_value_text_msg, &buf[offset], sizeof(char) * (len - offset));   
	request_msg.request_value_text_msg[len - offset] = '\0';
}

void ArmState::decodeTextFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, TextMessage &text_msg)
{
	memcpy(&text_msg.robot_msg_type, &buf[offset], sizeof(text_msg.robot_msg_type));   
	offset += sizeof(text_msg.robot_msg_type);

    memcpy(&text_msg.text_msg, &buf[offset], sizeof(char) * (len - offset));   
	text_msg.text_msg[len - offset] = '\0';
}

void ArmState::decodeRuntimeExceptFromRobotMsg(uint8_t* buf, unsigned int offset, int32_t len, 
    RuntimeExceptionMessage &runtime_except_msg)
{
	runtime_except_msg.line_num = *(uint32_t*)&buf[offset];
	offset += sizeof(runtime_except_msg.line_num);
	runtime_except_msg.line_num = ntohl(runtime_except_msg.line_num);

	runtime_except_msg.column_num = *(uint32_t*)&buf[offset];
	offset += sizeof(runtime_except_msg.column_num);
	runtime_except_msg.column_num = ntohl(runtime_except_msg.column_num);

	memcpy(&runtime_except_msg.text_msg, &buf[offset], sizeof(char) * (len - offset));   //array
	runtime_except_msg.text_msg[len - offset] = '\0';
}

