#include "control_arm.h"

bool ControlArm::getPopupWarning()
{
    return arm_comm_->arm_state_->getPopupWarning();
}

bool ControlArm::getPopupError()
{
    return arm_comm_->arm_state_->getPopupError();
}

bool ControlArm::getPopupBlocking()
{
    return arm_comm_->arm_state_->getPopupBlocking();
}

std::string ControlArm::getPopupTitle()
{
    return arm_comm_->arm_state_->getPopupTitle();
}

std::string ControlArm::getPopupText()
{
    return arm_comm_->arm_state_->getPopupText();
}

bool ControlArm::getRequestWarning()
{
    return arm_comm_->arm_state_->getRequestWarning();
}

bool ControlArm::getRequestError()
{
    return arm_comm_->arm_state_->getRequestError();
}

bool ControlArm::getRequestBlocking()
{
    return arm_comm_->arm_state_->getRequestBlocking();
}

std::string ControlArm::getRequesteTitle()
{
    return arm_comm_->arm_state_->getRequesteTitle(); 
}

std::string ControlArm::getRequestText()
{
    return arm_comm_->arm_state_->getRequestText(); 
}

std::string ControlArm::getText()
{
    return arm_comm_->arm_state_->getText(); 
}

uint32_t ControlArm::getRuntimeLineNum()
{
    return arm_comm_->arm_state_->getRuntimeLineNum(); 
}

uint32_t ControlArm::getRuntimeColumnNum()
{
    return arm_comm_->arm_state_->getRuntimeColumnNum(); 
}

std::string ControlArm::getRuntimeText()
{
    return arm_comm_->arm_state_->getRuntimeText(); 
}

std::string ControlArm::getVarTitle()
{
    return arm_comm_->arm_state_->getVarTitle(); 
}

std::string ControlArm::getVarText()
{
    return arm_comm_->arm_state_->getVarText(); 
}
