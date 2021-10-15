
#include "control_arm.h"

ControlArm::ControlArm(std::string host)
{
    arm_comm_ = new ArmComm(host);
}

ControlArm::~ControlArm()
{
    if(arm_comm_ != nullptr)
    {
        delete arm_comm_;
        arm_comm_ = nullptr;
    }
}

bool ControlArm::start()
{
    arm_comm_->arm_state_->setProtocolVersion(3.9);
    return arm_comm_->start();
}

void ControlArm::halt()
{
    arm_comm_->halt();
}

bool ControlArm::isConnected()
{
    arm_comm_->isRunning();
}