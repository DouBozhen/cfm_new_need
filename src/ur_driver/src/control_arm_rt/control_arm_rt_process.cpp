
#include "control_arm_rt.h"

double ControlArmRealtime::getTime()
{
    return arm_comm_rt_->arm_state_rt_->getTime();
}

ur_data_type::Joint ControlArmRealtime::getTargetJoint() const
{
	return arm_comm_rt_->arm_state_rt_->getTargetJoint();
}

ur_data_type::Joint ControlArmRealtime::getTargetJointVel() const
{
	return arm_comm_rt_->arm_state_rt_->getTargetJointVel();
}

ur_data_type::Joint ControlArmRealtime::getTargetJointAcc() const
{
	return arm_comm_rt_->arm_state_rt_->getTargetJointAcc();
}

ur_data_type::Joint ControlArmRealtime::getTargetCurrent() const
{
	return arm_comm_rt_->arm_state_rt_->getTargetCurrent();
}

ur_data_type::Joint ControlArmRealtime::getTargetTorque() const
{
	return arm_comm_rt_->arm_state_rt_->getTargetTorque();
}

ur_data_type::Joint ControlArmRealtime::getActualJoint() const
{
	return arm_comm_rt_->arm_state_rt_->getActualJoint();
}

ur_data_type::Joint ControlArmRealtime::getActualJointVel() const
{
	return arm_comm_rt_->arm_state_rt_->getActualJointVel();
}

ur_data_type::Joint ControlArmRealtime::getActualJointCurrent() const
{
	return arm_comm_rt_->arm_state_rt_->getActualJointCurrent();
}

ur_data_type::Joint ControlArmRealtime::getActualJointVoltage() const
{
	return arm_comm_rt_->arm_state_rt_->getActualJointVoltage();
}

ur_data_type::CartPose ControlArmRealtime::getActualToolPose() const
{
	// ur_data_type::CartPose data;
	// data.point.x = 0.035194;
	// data.point.y = -0.626302;
	// data.point.z = 0.387712;
	// data.rpy.rx = -3.072920;
	// data.rpy.ry = -0.455422;
	// data.rpy.rz = 0.026206;
	// return data;
	return arm_comm_rt_->arm_state_rt_->getActualToolPose();
}

ur_data_type::CartPose ControlArmRealtime::getActualTcpVel() const
{
	return arm_comm_rt_->arm_state_rt_->getActualTcpVel();
}

ur_data_type::CartPose ControlArmRealtime::getTcpForce() const
{
	return arm_comm_rt_->arm_state_rt_->getTcpForce();
}

ur_data_type::CartesianPoint ControlArmRealtime::getToolPointAcc() const
{
	return arm_comm_rt_->arm_state_rt_->getToolPointAcc();
}

uint64_t ControlArmRealtime::getDigitalInputs() const
{
	return arm_comm_rt_->arm_state_rt_->getDigitalInputs();
}

double ControlArmRealtime::getControllerTimer() const
{
	return arm_comm_rt_->arm_state_rt_->getControllerTimer();
}

ur_data_type::Joint ControlArmRealtime::getMotorTemperatures() const
{
	return arm_comm_rt_->arm_state_rt_->getMotorTemperatures();
}

bool ControlArmRealtime::setPayload(double pay_load)
{
	return arm_comm_rt_->setPayload(pay_load);
}

bool ControlArmRealtime::speedJointMove(ur_data_type::Joint joint_vel, double acc)
{
	return arm_comm_rt_->speedJointMove(joint_vel, acc);
}

bool ControlArmRealtime::jointMove(ur_data_type::Joint joint_position, 
	double acc, double vel, double during, double r)
{
	return arm_comm_rt_->jointMove(joint_position, acc, vel, during, r);
}

bool ControlArmRealtime::servoJ(ur_data_type::Joint joint_position, 
	double acc, double vel, double t1, double t2, int gain)
{
	return arm_comm_rt_->servoJ(joint_position, acc, vel, t1, t2, gain);
}

bool ControlArmRealtime::lineMove(ur_data_type::CartPose target_pose, 
	double acc, double vel, double during, double r)
{
	return arm_comm_rt_->lineMove(target_pose, acc, vel, during, r);
}

bool ControlArmRealtime::generateServoJScript(std::vector<ur_data_type::Joint> path, 
	double vel_min, double vel_max, std::string &script)
{
	return arm_comm_rt_->generateServoJScript(path, vel_min, vel_max, script);
}

bool ControlArmRealtime::sendScript(std::string script)
{
	return arm_comm_rt_->sendServoJScript(script);
}

bool ControlArmRealtime::stopJointMove()
{
	return arm_comm_rt_->stopJointMove();
}
bool ControlArmRealtime::initGlobalSpeed()
{
    return arm_comm_rt_->initGlobalSpeed();
}

bool ControlArmRealtime::pullUpPole()
{
	if(arm_comm_rt_->setDigitalOutput(1, false))
	{
		return arm_comm_rt_->setDigitalOutput(0, true);
	}
	return false;
}

bool ControlArmRealtime::pullDownPole()
{
	if(arm_comm_rt_->setDigitalOutput(0, false))
	{
		return arm_comm_rt_->setDigitalOutput(1, true);
	}
	return false;
}

bool ControlArmRealtime::ControlArmRealtime::delayPowerOff()
{
	return arm_comm_rt_->setDigitalOutput(4, true);
}

bool ControlArmRealtime::turnLightBeltRed(bool to_switch)
{
	return arm_comm_rt_->setDigitalOutput(5, to_switch);
}

bool ControlArmRealtime::turnLightBeltGreen(bool to_switch)
{
	return arm_comm_rt_->setDigitalOutput(6, to_switch);
}

bool ControlArmRealtime::turnLightBeltBlue(bool to_switch)
{
	return arm_comm_rt_->setDigitalOutput(7, to_switch);
}

bool ControlArmRealtime::turnEndRed(bool to_switch)
{
	return arm_comm_rt_->setDigitalOutput(8, to_switch);
}

bool ControlArmRealtime::turnEndGreen(bool to_switch)
{
	return arm_comm_rt_->setDigitalOutput(9, to_switch);
}


bool ControlArmRealtime::setDigitalOutput(unsigned int index, bool status)
{
	return arm_comm_rt_->setDigitalOutput(index, status);
}
bool ControlArmRealtime::setAnalogOutput(unsigned int index, double value)
{
	return arm_comm_rt_->setAnalogOutput(index, value);
}
bool ControlArmRealtime::setToolDigitalOutput(unsigned int index, bool status)
{
	return arm_comm_rt_->setToolDigitalOutput(index, status);
}