#include "control_arm.h"

char ControlArm::getToolAnalogInputRange(int index)
{
    return arm_comm_->arm_state_->getToolAnalogInputRange(index);
}

double ControlArm::getToolAnalogInput(int index)
{
    return arm_comm_->arm_state_->getToolAnalogInput(index);
}

float ControlArm::getToolVoltage48V()
{
    return arm_comm_->arm_state_->getToolVoltage48V();
}

uint8_t ControlArm::getToolOutputVoltage()
{
    return arm_comm_->arm_state_->getToolOutputVoltage();
}

float ControlArm::getToolCurrent()
{
    return arm_comm_->arm_state_->getToolCurrent();
}

float ControlArm::getToolTemperature()
{
    return arm_comm_->arm_state_->getToolTemperature();
}

uint8_t ControlArm::getToolMode()
{
    return arm_comm_->arm_state_->getToolMode();
}
