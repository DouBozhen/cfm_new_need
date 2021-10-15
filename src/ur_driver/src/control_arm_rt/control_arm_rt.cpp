
#include "control_arm_rt.h"

ControlArmRealtime::ControlArmRealtime(std::string host)
{
    arm_comm_rt_ = new ArmCommRealtime(host);
}

ControlArmRealtime::~ControlArmRealtime()
{
    if(arm_comm_rt_ != nullptr)
    {
        delete arm_comm_rt_;
        arm_comm_rt_ = nullptr;
    }
}

bool ControlArmRealtime::start()
{
    arm_comm_rt_->arm_state_rt_->setProtocolVersion(3.9);
    string version = getVersion();
    printf("UR Driver Software Version: %s\n", version.c_str());
    return arm_comm_rt_->start();
}

void ControlArmRealtime::halt()
{
    arm_comm_rt_->halt();
    printf("ControlArmRealtime: exit.\n");
}

std::string ControlArmRealtime::getVersion()
{
    return "2.0.1.20210602";
}

bool ControlArmRealtime::isConnected()
{
    return arm_comm_rt_->isConnected();
}