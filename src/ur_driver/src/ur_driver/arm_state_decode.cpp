#include <arpa/inet.h>
#include <stdio.h>
#include "arm_state.h"

double ArmState::ntohd(uint64_t nf)
{
	double x;
	nf = be64toh(nf);  //大端到主机字节序转换
	x = *(double *)&nf;
	return x;
}

float ArmState::ntohf(uint32_t nf)
{
	nf = be32toh(nf);    //大端到主机字节序转换
	float x = *(float *)&nf;
	return x;
}

void ArmState::decode(uint8_t* buf, unsigned int len)
{
    // general_pkg_.msg_type = static_cast<message_types::MessageType>(buf[sizeof(int)]);
    general_pkg_.msg_type = static_cast<message_types::MessageType>(buf[0]);

    switch (general_pkg_.msg_type)
    {
        case message_types::ROBOT_MESSAGE:    // 16
            decodeRobotMsg(buf, 1, len); 
            break;
        case message_types::ROBOT_STATE:     // 20
            decodeRobotStateMsg(buf, 1, len); 
            break;
        case message_types::PROGRAM_STATE_MESSAGE:   // 25
            // decodeProgStateMsg(buf, 1, len); 
            break;
        default:
            break;
    }
}

void ArmState::decodeRobotMsg(uint8_t* buf, unsigned int offset, int32_t len)
{
	uint64_t time_stamp = *(uint64_t*)&buf[offset]; 
    time_stamp = ntohl(time_stamp);
    offset += sizeof(uint64_t);

	int8_t source = buf[offset];
    offset += 1;

	robot_msg_pkg_.robot_msg_type = static_cast<RobotMessageType>(buf[offset]);
    offset += 1;

	switch (robot_msg_pkg_.robot_msg_type)
	{
        case RobotMessageType::ROBOT_MESSAGE_VERSION:
        {
            version_msg_.time_stamp = time_stamp;
            version_msg_.source = source;
            version_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeVersionFromRobotMsg(buf, offset, len, version_msg_);
            if (version_msg_.major_version < 2) // to do...
            {
                robot_mode_running_ = message_types::ROBOT_RUNNING_MODE;
            }
        }
		break;
        case RobotMessageType::ROBOT_MESSAGE_SAFETY_MODE:
        {
            safety_mode_msg_.time_stamp = time_stamp;
            safety_mode_msg_.source = source;
            safety_mode_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeSafetyModeFromRobotMsg(buf, offset, len, safety_mode_msg_);
        }
        break;
        case RobotMessageType::ROBOT_MESSAGE_KEY:
        {
            key_msg_.time_stamp = time_stamp;
            key_msg_.source = source;
            key_msg_.key_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeKeyFromRobotMsg(buf, offset, len, key_msg_);
        }
		break;
        case RobotMessageType::ROBOT_MESSAGE_ERROR_CODE:
        {
            robot_comm_msg_.time_stamp = time_stamp;
            robot_comm_msg_.source = source;
            robot_comm_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeRobotCommFromRobotMsg(buf, offset, len, robot_comm_msg_);
        }
		break;
	    case RobotMessageType::ROBOT_MESSAGE_PROGRAM_LABEL:
        {
            lable_msg_.time_stamp = time_stamp;
            lable_msg_.source = source;
            lable_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeLabelFromRobotMsg(buf, offset, len, lable_msg_);
        }
		break;
        case RobotMessageType::ROBOT_MESSAGE_TYPE_POPUP:
        {
            popup_msg_.time_stamp = time_stamp;
            popup_msg_.source = source;
            popup_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodePopupFromRobotMsg(buf, offset, len, popup_msg_);
        }
		break;
        case RobotMessageType::ROBOT_MESSAGE_REQUEST_VALUE:
        {
            request_msg_.time_stamp = time_stamp;
            request_msg_.source = source;
            request_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeRequestFromRobotMsg(buf, offset, len, request_msg_);
        }
		break;
        case RobotMessageType::ROBOT_MESSAGE_TEXT:
        {
            text_msg_.time_stamp = time_stamp;
            text_msg_.source = source;
            text_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeTextFromRobotMsg(buf, offset, len, text_msg_);
        }
		break;
        case RobotMessageType::ROBOT_MESSAGE_RUNTIME_EXCEPTION:
        {
            runtime_except_msg_.time_stamp = time_stamp;
            runtime_except_msg_.source = source;
            runtime_except_msg_.robot_msg_type = robot_msg_pkg_.robot_msg_type;
            decodeRuntimeExceptFromRobotMsg(buf, offset, len, runtime_except_msg_);
        }
		break;
	    default:
            printf("ArmState: robot msg type error.\n");
		break;
	}
}

void ArmState::decodeRobotStateMsg(uint8_t* buf, unsigned int offset, uint32_t len)
{
    while (offset < len)
    {
        int32_t length = *(int32_t*)&buf[offset];
        length = ntohl(length);
        offset += sizeof(int32_t);

        RobotStateMessageType robot_state_msg_type = static_cast<RobotStateMessageType>(buf[offset]);
        offset += 1;

        switch (robot_state_msg_type)
        {
            case message_types::ROBOT_MODE_DATA:
                decodeRobotModeFromRobotStateMsg(buf, offset, robot_mode_);
                break;
            case message_types::JOINT_DATA:
                decodeJointdataFromRobotStateMsg(buf, offset, joint_data_);
                break;
            case message_types::CARTESIAN_INFO:
                decodeCartInfoFromRobotStateMsg(buf, offset, cart_info_);
                break;
            case message_types::MASTERBOARD_DATA:
                decodeMasterboardFromRobotStateMsg(buf, offset, mb_data_);
                break;
            case message_types::TOOL_DATA:
                decodeToolDataFromRobotStateMsg(buf, offset, tool_data_);
                break;
            case message_types::KINEMATICS_INFO:
                decodeKineInfoFromRobotStateMsg(buf, offset, kine_info_);
                break;
            case message_types::CONFIGURATION_DATA:
                decodeConfigDataFromRobotStateMsg(buf, offset, config_data_);
                break;
            case message_types::FORCE_MODE_DATA:
                decodeForceModeFromRobotStateMsg(buf, offset, force_mode_);
                break;
            case message_types::ADDITIONAL_INFO:
                decodeAdditionalInfoFromRobotState(buf, offset, additional_info_);
                break;
            case message_types::CALIBRATION_DATA:
                break;
            default:
                break;
        }
        offset += length - 5;
        new_data_available_ = true;
    }
}


void ArmState::decodeProgStateMsg(uint8_t* buf, unsigned int offset, int32_t len)
{
	uint64_t time_stamp = *(uint64_t*)&buf[offset];
    offset += sizeof(uint64_t);

	int8_t source = buf[offset];
    offset += 1;

	program_state_msg_pkg_.program_state_msg_type = static_cast<ProgramStateMessageType>(buf[offset]);
    offset += 1;

	switch (program_state_msg_pkg_.program_state_msg_type)
	{
        case message_types::PROGRAM_STATE_MESSAGE_VARIABLE_UPDATE:
        {
            var_msg_.time_stamp = time_stamp;
            var_msg_.robot_msg_type = program_state_msg_pkg_.program_state_msg_type;
            decodeVarFromProgStateMsg(buf, offset, len, var_msg_);
        }
		    break;
        case message_types::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_SETUP:
        {
            global_setup_msg_.time_stamp = time_stamp;
            global_setup_msg_.robot_msg_type = program_state_msg_pkg_.program_state_msg_type;
            decodeGlobalParamsSetupFromProgStateMsg(buf, offset, len, global_setup_msg_);
        }
		    break;
        case message_types::PROGRAM_STATE_MESSAGE_GLOBAL_VARIABLES_UPDATE:
        {   
            global_update_msg_.time_stamp = time_stamp;
            global_update_msg_.robot_msg_type = program_state_msg_pkg_.program_state_msg_type;
            decodeGlobalParamsUpdataFromProgStateMsg(buf, offset, len, global_update_msg_);
        }
		    break;
        default:
            break;
	}
}
