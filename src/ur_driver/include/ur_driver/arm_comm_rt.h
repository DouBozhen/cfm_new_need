#ifndef ARM_COMM_REALTIME_H
#define ARM_COMM_REALTIME_H

#include <string>
#include <vector>
#include "thread_base.h"
#include "tcp_client.h"

#include "arm_state_rt.h"
#include "ur_datatype.h"

#define RT_PORT 30003
#define SAFETY_COUNT_MAX 12
#define TCP_COMM_PACKAGE_LEN 1108
#define TCP_COMM_BUF_SIZE 11080
#define TCP_CONNECT_MAX_COUNT 10

#define PAY_LOAD_MIN (0.0)
#define PAY_LOAD_MAX (5.0)

class ArmCommRealtime : public ThreadBase<ArmCommRealtime>
{
public:
	ArmCommRealtime(std::string host);
    ~ArmCommRealtime();

    bool start();
	void halt();
	void run();
	bool isConnected();

	bool connected_;
	ArmStateRealtime* arm_state_rt_;

	bool setPayload(double pay_load);

	bool speedJointMove(ur_data_type::Joint joint_vel, double acc = 100.0);
	bool jointMove(ur_data_type::Joint joint_position, double acc, double vel, double during, double r);
	bool servoJ(ur_data_type::Joint joint_position, double acc, double vel, double t1, double t2, int gain);
	bool lineMove(ur_data_type::CartPose target_pose, double acc, double vel, double during, double r);
	bool stopJointMove();
	bool generateServoJScript(std::vector<ur_data_type::Joint> path, double vel_min, double vel_max, std::string &script);
	bool sendServoJScript(std::string script);
    bool initGlobalSpeed();

	bool setToolVoltage(unsigned int voltage);
	bool setFlag(unsigned int index, bool status);

	bool setDigitalOutput(unsigned int index, bool status); 
	bool setAnalogOutput(unsigned int index, double value);
	bool setToolDigitalOutput(unsigned int index, bool status);

private:
	unsigned int safety_count_;
    unsigned int safety_count_max_;

	// bool is_running_;

	double minimum_payload_ = 0;
	double maximum_payload_ = 5;

    int recv_bytes_;
    uint8_t buf_[TCP_COMM_BUF_SIZE];

    int connect_times_;
    std::string host_;
    TcpClient* tcp_client_;

    long long getCurrentTime();
	void rtNotifyCall();

 	bool recvMessage(std::string& recv_msg);
    bool sendMessage(std::string send_msg);
};

#endif