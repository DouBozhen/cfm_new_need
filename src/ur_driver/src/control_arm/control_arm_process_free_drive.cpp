#include "control_arm.h"

bool ControlArm::isFreeDrivePressed()
{
    return arm_comm_->arm_state_->isFreeDrivePressed();
}

bool ControlArm::isFreeDriveEnabled()
{
    return arm_comm_->arm_state_->isFreeDriveEnabled();
}

bool ControlArm::isFreeDriveIOEnabled()
{
    return arm_comm_->arm_state_->isFreeDriveIOEnabled();
}

