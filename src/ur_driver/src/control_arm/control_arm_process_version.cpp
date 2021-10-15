#include <stdio.h>
#include "control_arm.h"
using namespace std;

int8_t ControlArm::getSource()
{
    return arm_comm_->arm_state_->getSource();
}

std::string ControlArm::getProjectName()
{
    return arm_comm_->arm_state_->getProjectName();
}

std::string ControlArm::getVersion()
{
    return arm_comm_->arm_state_->getVersion();
}

std::string ControlArm::getBugfixVersion()
{
    return arm_comm_->arm_state_->getBugfixVersion();
}
