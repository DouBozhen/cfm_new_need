#include "control_arm.h"


bool ControlArm::getDigitalInputs(int index) // ????, to do...
{
    int inputs = arm_comm_->arm_state_->getDigitalInputs();
	int8_t lower_8bits_in_low_word = inputs & 0xFF; //低字中的低八位
	int8_t lower_8bits_in_high_word  = (inputs >> 16) & 0xFF;//高字中的低八位， 用于工具数字输入的17、18位
	int digtal_bits = 0;
	int status = 0;
	if (index < 16)
	{
		status = lower_8bits_in_low_word & (1 << index);
	}
	else
	{
		status = lower_8bits_in_high_word & (1 << (index - 16));
	}

    if(status == 0) return false;
    else return true;
}

int ControlArm::getDigitalOutputs()
{
    return arm_comm_->arm_state_->getDigitalOutputs();
}

char ControlArm::getAnalogInputRange(int index)
{
    return arm_comm_->arm_state_->getAnalogInputRange(index);
}

double ControlArm::getAnalogInput(int index)
{
    return arm_comm_->arm_state_->getAnalogInput(index);
}

char ControlArm::getAnalogOutputDomain(int index)
{
    return arm_comm_->arm_state_->getAnalogOutputDomain(index);
}

double ControlArm::getAnalogOutput(int index)
{
    return arm_comm_->arm_state_->getAnalogOutput(index);
}

float ControlArm::getMasterBoardTemperature()
{
    return arm_comm_->arm_state_->getMasterBoardTemperature();
}   

float ControlArm::getRobotVoltage48V()             
{
    return arm_comm_->arm_state_->getRobotVoltage48V();
}

float ControlArm::getRobotCurrent()
{
    return arm_comm_->arm_state_->getRobotCurrent();
}

float ControlArm::getMasterIOCurrent()
{
    return arm_comm_->arm_state_->getMasterIOCurrent();
}

uint8_t ControlArm::getSafetyMode()          
{
    return arm_comm_->arm_state_->getSafetyMode();
}

uint8_t ControlArm::getInReducedMode()
{
    return arm_comm_->arm_state_->getInReduceMode();
}

int8_t ControlArm::getEuromap67Installed()     
{
    return arm_comm_->arm_state_->isEuromap67Installed();
}

int ControlArm::getEuromapInputs()
{
    return arm_comm_->arm_state_->getEuromapInputs();
}

int ControlArm::getEuromapOutputs()
{
    return arm_comm_->arm_state_->getEuromapOutputs();
}

float ControlArm::getEuromapVoltage()
{
    return arm_comm_->arm_state_->getEuromapVoltage();
}

float ControlArm::getEuromapCurrent()
{
    return arm_comm_->arm_state_->getEuromapCurrent();
}

uint8_t ControlArm::getOperationModeInput()
{
    return arm_comm_->arm_state_->getOpModeInput();
}

uint8_t ControlArm::getThreePositionEnablingInput()
{
    return arm_comm_->arm_state_->get3PositionEnablingSwitch();
}
