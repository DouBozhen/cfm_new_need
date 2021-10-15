
#include "control_arm.h"

bool ControlArm::isRobotConnected()
{
    return arm_comm_->arm_state_->isRobotConnected();
}

bool ControlArm::isRobotEnabled()
{
    return arm_comm_->arm_state_->isRobotEnabled();
}

bool ControlArm::isRobotPoweredOn()
{
    return arm_comm_->arm_state_->isRobotPoweredOn();
}

bool ControlArm::isEmergencyStopped()
{
    return arm_comm_->arm_state_->isEmergencyStopped();
}

bool ControlArm::isProtectiveStopped()
{
    return arm_comm_->arm_state_->isProtectiveStopped();
}

bool ControlArm::isProgramRunning()
{
    return arm_comm_->arm_state_->isProgramRunning();
}

bool ControlArm::isProgramPaused()
{
    return arm_comm_->arm_state_->isProgramPaused();
}

uint8_t ControlArm::getRobotMode()            
{
    return arm_comm_->arm_state_->getRobotMode();
}

uint8_t ControlArm::getControlMode()        
{
    return arm_comm_->arm_state_->getControlMode();
}

double ControlArm::getTargetVelFraction()
{
    return arm_comm_->arm_state_->getTargetSpeedFraction();
}

double ControlArm::getVelScaling()
{
    return arm_comm_->arm_state_->getSpeedScaling();
}

double ControlArm::getTargetVelFractionLimit()
{
    return arm_comm_->arm_state_->getTargetSpeedFractionLimit();
}

ur_data_type::Joint ControlArm::getActualJoint()
{
    return arm_comm_->arm_state_->getActualJoint();
}

ur_data_type::Joint ControlArm::getTargetJoint()
{
   return arm_comm_->arm_state_->getTargetJoint();
}

ur_data_type::Joint ControlArm::getActualJointSpeed()
{
    return arm_comm_->arm_state_->getActualJointVel();
}

ur_data_type::Joint ControlArm::getActualJointCurrent()
{
    return arm_comm_->arm_state_->getActualJointCurrent();
}

ur_data_type::Joint ControlArm::getActualJointVoltage()
{
    return arm_comm_->arm_state_->getActualJointVoltage();
}

ur_data_type::Joint ControlArm::getTMotor()
{
    return arm_comm_->arm_state_->getTmotor();
}

ur_data_type::Joint ControlArm::getTMicro()
{
    return arm_comm_->arm_state_->getTmicro();
}

std::array<unsigned char, 6> ControlArm::getAllJointMode()
{
    return arm_comm_->arm_state_->getAllJointMode();
}

ur_data_type::CartPose ControlArm::getToolPose()
{
    return arm_comm_->arm_state_->getToolPose();
}

double ControlArm::getToolPointX()
{
    return arm_comm_->arm_state_->getToolPointX();
}

double ControlArm::getToolPointY()
{
    return arm_comm_->arm_state_->getToolPointY();
}

double ControlArm::getToolPointZ()
{
    return arm_comm_->arm_state_->getToolPointZ();
}

double ControlArm::getToolRpyRx()
{
    return arm_comm_->arm_state_->getRpyRx();
}

double ControlArm::getToolRpyRy()
{
    return arm_comm_->arm_state_->getRpyRy();
}

double ControlArm::getToolRpyRz()
{
    return arm_comm_->arm_state_->getRpyRz();
}

ur_data_type::CartPose ControlArm::getTcpOffsetPose()
{
    return arm_comm_->arm_state_->getTcpOffsetPose();
}

double ControlArm::getTcpOffsetPointX()
{
    return arm_comm_->arm_state_->getTcpOffsetPointX();
}

double ControlArm::getTcpOffsetPointY()
{
    return arm_comm_->arm_state_->getTcpOffsetPointY();
}

double ControlArm::getTcpOffsetPointZ()
{
    return arm_comm_->arm_state_->getTcpOffsetPointZ();
}

double ControlArm::getTcpOffsetRpyRx()
{
    return arm_comm_->arm_state_->getTcpOffsetRpyRx();
}

double ControlArm::getTcpOffsetRpyRy()
{
    return arm_comm_->arm_state_->getTcpOffsetRpyRy();
}

double ControlArm::getTcpOffsetRpyRz()
{
    return arm_comm_->arm_state_->getTcpOffsetRpyRz();
}

