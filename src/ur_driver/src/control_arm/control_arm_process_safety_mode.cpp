#include "control_arm.h" 

int8_t ControlArm::getSafetyModeType()
{
    return arm_comm_->arm_state_->getSafetyModeType();
}

std::string ControlArm::getSafetyTextMsg()
{
    return arm_comm_->arm_state_->getSafetyTextMsg();
}

int ControlArm::getRobotCommWarningLevel()
{
    return arm_comm_->arm_state_->getRobotCommWarningLevel();
}

std::string ControlArm::getRobotCommText()
{
    return arm_comm_->arm_state_->getRobotCommText();
}