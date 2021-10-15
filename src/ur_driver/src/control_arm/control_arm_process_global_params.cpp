#include "control_arm.h"
using namespace std;
using namespace ur_data_type;

uint16_t ControlArm::getSetupGlobalParamsStartIndex()
{
    return arm_comm_->arm_state_->getSetupGlobalParamsStartIndex();
}
std::vector<std::string> ControlArm::getSetupGlobalParamsNames()
{
    return arm_comm_->arm_state_->getSetupGlobalParamsNames();
}
	
uint8_t ControlArm::getUpdateGlobalParamsStartIndex()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsStartIndex();
}

uint8_t ControlArm::getUpdateGlobalParamsValueType()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsValueType();
}

std::string ControlArm::getUpdateGlobalParamsStringValue()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsStringValue();
}

std::string ControlArm::getUpdateGlobalParamsStringValueVar()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsStringValueVar();
}

int16_t ControlArm::getUpdateGlobalParamsListLength()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsListLength();
}
uint8_t ControlArm::getUpdateGlobalParamsListValType()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsListValType();
}

ur_data_type::CartPose ControlArm::getUpdateGlobalParamsPose()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsPose();
}

bool ControlArm::getUpdateGlobalParamsBool()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsBool();
}

float ControlArm::getUpdateGlobalParamsNum()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsNum();
}

int32_t ControlArm::getUpdateGlobalParamsInt()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsInt();
}

float ControlArm::getUpdateGlobalParamsFloat()
{
    return arm_comm_->arm_state_->getUpdateGlobalParamsFloat();
}
