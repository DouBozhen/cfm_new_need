#include "control_arm.h"

ur_data_type::CartPose ControlArm::getForce()
{
    return arm_comm_->arm_state_->getForce();
}

double ControlArm::getForcePointX()
{
    return arm_comm_->arm_state_->getForcePointX();
}

double ControlArm::getForcePointY()
{
    return arm_comm_->arm_state_->getForcePointY();
}

double ControlArm::getForcePointZ()
{
    return arm_comm_->arm_state_->getForcePointZ();
}

double ControlArm::getForceRpyRx()
{
    return arm_comm_->arm_state_->getForceRpyRx();
}

double ControlArm::getForceRpyRy()
{
    return arm_comm_->arm_state_->getForceRpyRy();
}

double ControlArm::getForceRpyRz()
{
    return arm_comm_->arm_state_->getForceRpyRz();
}

double ControlArm::getForceDexterity()
{
    return arm_comm_->arm_state_->getForceDexterity();
}
