#include "control_arm.h"

std::string ControlArm::getKeyTitle()
{
    return arm_comm_->arm_state_->getKeyTitle();
}

std::string ControlArm::getKeyText()
{
    return arm_comm_->arm_state_->getKeyText();
}

int ControlArm::getLabelId()
{
    return arm_comm_->arm_state_->getLabelId();
}
std::string ControlArm::getLabelText()
{
    return arm_comm_->arm_state_->getLabelText();
}