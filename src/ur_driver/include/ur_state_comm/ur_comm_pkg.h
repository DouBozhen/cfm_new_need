#ifndef UR_COMM_PACKAGE_H
#define UR_COMM_PACKAGE_H

#include "ur_comm_msg_type.h"

struct GeneralPackage
{
	message_types::MessageType msg_type;
};

struct RobotMessagePackage
{
	message_types::RobotMessageType robot_msg_type;
};

struct ProgramStateMessagePackage
{
	message_types::ProgramStateMessageType program_state_msg_type;
};

#endif