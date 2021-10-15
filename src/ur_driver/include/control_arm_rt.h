#ifndef CONTROL_ARM_REALTIME_H
#define CONTROL_ARM_REALTIME_H

#include <string>
#include <vector>
#include "ur_datatype.h"
#include "arm_comm_rt.h"

class ControlArmRealtime 
{
public:
    ControlArmRealtime(std::string host);
    ~ControlArmRealtime();

    bool start();
    void halt();
	bool isConnected();

	std::string getVersion();
    double getTime();

	ur_data_type::Joint getTargetJoint() const;
    ur_data_type::Joint getTargetJointVel() const;
    ur_data_type::Joint getTargetJointAcc() const;
    ur_data_type::Joint getTargetCurrent() const;
    ur_data_type::Joint getTargetTorque() const;

    ur_data_type::Joint getActualJoint() const;
    ur_data_type::Joint getActualJointVel() const; 
    ur_data_type::Joint getActualJointCurrent() const; 
	ur_data_type::Joint getActualJointVoltage() const;

    ur_data_type::CartPose getActualToolPose() const;
    ur_data_type::CartPose getActualTcpVel() const;
    ur_data_type::CartPose getTcpForce() const;
    ur_data_type::CartesianPoint getToolPointAcc() const; 

    uint64_t getDigitalInputs() const;

	double getControllerTimer() const; 

	ur_data_type::Joint getMotorTemperatures() const;

	bool setPayload(double pay_load);

	bool speedJointMove(ur_data_type::Joint joint_vel, double acc);
	bool jointMove(ur_data_type::Joint joint_position, double acc, double vel, double during, double r); //r:?
	bool servoJ(ur_data_type::Joint joint_position, double acc, double vel, double t1, double t2, int gain);
	bool lineMove(ur_data_type::CartPose target_pose, double acc, double vel, double during, double r); //r:?
	bool stopJointMove();
	bool initGlobalSpeed();

	bool generateServoJScript(std::vector<ur_data_type::Joint> path, 
		double vel_min, double vel_max, std::string &script);
	bool sendScript(std::string script);

	bool pullUpPole();
	bool pullDownPole();

	bool turnLightBeltRed(bool to_switch);
	bool turnLightBeltGreen(bool to_switch);
	bool turnLightBeltBlue(bool to_switch);	

	bool turnEndGreen(bool to_switch); 
	bool turnEndRed(bool to_switch); 

	bool setDigitalOutput(unsigned int index, bool status); 
	bool setAnalogOutput(unsigned int index, double value);
	bool setToolDigitalOutput(unsigned int index, bool status);

private:
	ArmCommRealtime* arm_comm_rt_;
	bool delayPowerOff();
};

#endif
