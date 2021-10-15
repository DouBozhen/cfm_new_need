#ifndef ARM_STATE_REALTIME_H
#define ARM_STATE_REALTIME_H

#include <array>
#include <stdint.h>
#include <pthread.h>
#include "ur_datatype.h"

class ArmStateRealtime 
{
public:
    ArmStateRealtime();
    ~ArmStateRealtime();

    double getProtocolVersion() const;
    void setProtocolVersion(double version); 
	double getTime(); 

	ur_data_type::Joint getTargetJoint() const; 
    ur_data_type::Joint getTargetJointVel() const; 
    ur_data_type::Joint getTargetJointAcc() const; 
    ur_data_type::Joint getTargetCurrent() const; 
    ur_data_type::Joint getTargetTorque() const; 

    ur_data_type::Joint getActualJoint() ; 
    ur_data_type::Joint getActualJointVel() const;  
    ur_data_type::Joint getActualJointCurrent() const;  
    ur_data_type::Joint getControlJointCurrent() const; // to do...
    ur_data_type::Joint getActualJointVoltage() const;  // to do...

    ur_data_type::CartPose getActualToolPose() const; 
    ur_data_type::CartPose getActualTcpVel() const; 
    ur_data_type::CartPose getTcpForce() const; 
    ur_data_type::CartPose getTargetToolPose() const;// not decode
    ur_data_type::CartPose getTargetTcpVel() const; //to do...
    ur_data_type::CartesianPoint getToolPointAcc() const; 

	ur_data_type::CartesianPoint getElbowPoint() const; // not decode
	ur_data_type::CartesianPoint getElbowVel() const; // not decode
	double getLinearMomentumNorm() const; // to do...
	double getSpeedScaling() const; // to do...

    uint64_t getDigitalInputs() const; 
    uint64_t getDigitalOutputs() const;  // not decode

	uint64_t getRobotMode() const; // to do...
    ur_data_type::Joint getAllJointMode() const; // to do...
	uint64_t getSafetyMode() const; // to do...
    uint64_t getProgramState() const; // not decode

	double getControllerTimer() const; 

	ur_data_type::Joint getMotorTemperatures() const; 
	double getMainVoltage() const; // to do...
	double getRobotVoltage() const; // to do...
	double getRobotCurrent() const; // to do...

	void decode(uint8_t* buf);

private:
	double protocol_version_;                       
	double time_;                                   
	ur_data_type::Joint joint_position_;              
	ur_data_type::Joint joint_vel_;                  
	ur_data_type::Joint joint_acc_;                  
	ur_data_type::Joint joint_current_;               
	ur_data_type::Joint joint_torque_;                


	ur_data_type::Joint actual_joint_position_;       
	ur_data_type::Joint actual_joint_vel_;            
	ur_data_type::Joint actual_current_;              
	ur_data_type::Joint control_current_;             

	ur_data_type::CartPose actual_tool_pose_;        
	ur_data_type::CartPose actual_tcp_vel_;           
	ur_data_type::CartPose tcp_force_;                
	ur_data_type::CartPose target_tool_pose_;         
	ur_data_type::CartPose target_tcp_vel_;           
	ur_data_type::CartesianPoint tool_point_acc_;       

	uint64_t digital_input_;                        
    uint64_t digital_outputs_;                      

	double robot_mode_;                        
	ur_data_type::Joint joint_control_modes_; 
	double safety_mode_;        
    double program_state_;

	double controller_timer_;                     
	double vel_scaling_; 
	double linear_momentum_norm_;  
    ur_data_type::CartesianPoint elbow_point_; 
    ur_data_type::CartesianPoint elbow_vel_; 

	double main_voltage_;   
	double robot_voltage_; 
	double robot_current_; 
	ur_data_type::Joint joint_voltage_;   
	ur_data_type::Joint motor_temperatures_;         

	double ntohd(uint64_t data); //uint64_t to double ??

	template <class T,size_t N>
	inline void decodeVector(uint8_t* buf, int& start_index, int length, std::array<T, N>& vec)
	{
		for (int i = 0; i < vec.size(); i++)
		{
			vec[i] = ntohd(*(uint64_t *)&buf[start_index + i * sizeof(T)]);
		}
	}

};

#endif