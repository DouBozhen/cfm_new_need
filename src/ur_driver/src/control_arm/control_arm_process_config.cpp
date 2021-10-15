#include "control_arm.h"

ur_data_type::Joint ControlArm::getJointLimitMin()
{
    return arm_comm_->arm_state_->getJointLimitMin();
}

ur_data_type::Joint ControlArm::getJointLimitMax()
{
    return arm_comm_->arm_state_->getJointLimitMax();
}

ur_data_type::Joint ControlArm::getJointSpeedMax()
{
    return arm_comm_->arm_state_->getJointMaxVel();
}

ur_data_type::Joint ControlArm::getJointAccMax()
{
    return arm_comm_->arm_state_->getJointMaxAcc();
}

double ControlArm::getJointVDefault()
{
    return arm_comm_->arm_state_->getJointVDefault();
}

double ControlArm::getJointADefault()
{
    return arm_comm_->arm_state_->getJointADefault();
}

double ControlArm::getToolVDefault()
{
    return arm_comm_->arm_state_->getToolVDefault();
}

double ControlArm::getToolADefault()
{
    return arm_comm_->arm_state_->getToolADefault();
}

double ControlArm::getEqRad()
{
    return arm_comm_->arm_state_->getEqRad();
}

ur_data_type::DHParam ControlArm::getKineDHParam()
{
    return arm_comm_->arm_state_->getKineDHParam();
}

int ControlArm::getMasterBoardVersion()
{
    return arm_comm_->arm_state_->getMasterBoardVersion();
}

int ControlArm::getControllerBoxType()
{
    return arm_comm_->arm_state_->getControllerBoxType();
}

int ControlArm::getRobotType()
{
    return arm_comm_->arm_state_->getRobotType();
}

int ControlArm::getRobotSubType()
{
    return arm_comm_->arm_state_->getRobotSubType();
}
