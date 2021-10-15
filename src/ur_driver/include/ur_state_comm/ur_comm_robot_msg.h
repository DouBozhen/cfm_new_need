#ifndef UR_COMM_MESSAGE_H
#define UR_COMM_MESSAGE_H

#include <stdint.h>
#include "ur_datatype.h"

struct VersionMessage
{
	uint64_t time_stamp;
	int8_t source;
	int8_t robot_msg_type;
	int8_t project_name_size;
	char project_name[15];
	uint8_t major_version;
	uint8_t minor_version;
	int svn_revision; // bug_fix_version
	int build_number;
	char build_date[25];
};

struct SafetyModeMessage
{
	uint64_t time_stamp;
	int8_t  source;
	int8_t robot_msg_type;
	int robot_msg_code;
	int robot_msg_arg;
	int8_t safety_mode_type;
	int32_t report_data_type;
	uint32_t report_data;
	char text_msg[512];
};

struct RobotCommMessage
{
	uint64_t time_stamp;
	int8_t  source;
	int8_t robot_msg_type;
	int robot_msg_code;
	int robot_msg_arg;
	int warning_level;
	uint32_t robot_msg_data_type;
	uint32_t robot_msg_data;
	char text_msg[512];
};

struct KeyMessage
{
	uint64_t time_stamp;
	int8_t source;
	int8_t key_msg_type;
	int key_msg_code;
	int key_msg_arg;
	unsigned char key_title_size;
	char key_msg_title[512];
	char key_text_msg[512];
};

struct LabelMessage
{
	uint64_t time_stamp;
	int8_t source;
	int8_t robot_msg_type;
	int id;
	char label_text_msg[1024];
};

struct PopupMessage
{
	uint64_t time_stamp;
	int8_t  source;
	int8_t robot_msg_type;
	unsigned int request_id;
	unsigned char request_type;
	bool warning;
	bool error;
	bool blocking;
	uint8_t popup_msg_title_size;
	char popup_msg_title[512];
	char popup_text_msg[1024];
};

struct RequestValueMessage
{
	uint64_t time_stamp;
	int8_t  source;
	int8_t robot_msg_type;
	unsigned int request_id;
	unsigned char request_type;
	bool warning;
	bool error;
	bool blocking;
	unsigned char title_length;
	char decode_vector_title[1024];
	char request_value_text_msg[1024];
};

struct TextMessage
{
	uint64_t time_stamp;
	int8_t  source;
	int8_t robot_msg_type;
	char  text_msg[1024];
};

struct RuntimeExceptionMessage
{
	uint64_t time_stamp;
	int8_t  source;
	int8_t robot_msg_type;
	uint32_t line_num;
	uint32_t column_num;
	char text_msg[1024];
};

struct VarMessage
{
	uint64_t time_stamp;
	int8_t robot_msg_type;
	unsigned char title_size;
	char  msg_title[1024];
	char  msg_text[1024];
};

struct GlobalParamsSetupMessage
{
	uint64_t time_stamp;
	int8_t robot_msg_type;
	unsigned short start_index;
	char  param_names[1024];
};

struct GlobalParamsUpdateMessage
{
	uint64_t time_stamp;
	int8_t robot_msg_type;
	unsigned char start_index;
	uint8_t value_type;

	int16_t string_len;
	char string_value[1024];

	int16_t var_string_len;
	char var_string_value[1024];
	ur_data_type::CartPose pose;

	bool bool_val;
	int32_t int_val;
	float  num_value;
	float float_val;

	int16_t list_length;
	unsigned char list_val_type;
};

#endif
