#include "controller.h"

bool Controller::getGpioSensorStatus()
{
    uint64_t inputs = control_arm_rt_->getDigitalInputs();
    uint64_t all_inputs = (inputs & 0x10) >> 4;
    bool status = static_cast<bool>(all_inputs);
    // printf("status = %d, inputs = 0x%08x, all_inputs = 0x%08x\n", status, inputs, all_inputs);
    return status;
}

bool Controller::setLampBeltGreen()
{
    if(control_arm_rt_->setDigitalOutput(INPUT_INDEX_LAMB_BELT_BLUE, false))
        return control_arm_rt_->setDigitalOutput(INPUT_INDEX_LAMB_BELT_GREEN, true); 
    return false;
}

bool Controller::setLampBeltBlue()
{
    if(control_arm_rt_->setDigitalOutput(INPUT_INDEX_LAMB_BELT_GREEN, false))
	    return control_arm_rt_->setDigitalOutput(INPUT_INDEX_LAMB_BELT_BLUE, true); 
    return false;
}
