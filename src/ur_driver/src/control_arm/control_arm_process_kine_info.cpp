#include "control_arm.h"

std::array<uint32_t, 6> ControlArm::getKinematicsChecksum()
{
    return arm_comm_->arm_state_->getKineCheckSum();
}

#if 0
ur_data_type::DHParam ControlArm::getKineDHParam()
{
    return arm_comm_->arm_state_->getKineDHParam();
}
#endif
