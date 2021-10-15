#include <unistd.h>
#include <math.h>
#include <cstring>
#include <vector>
#include "test_control_arm_rt.h"

using namespace ur_data_type;
using namespace std;

TestControlArmRT::TestControlArmRT(std::string ip)
{
    control_arm_rt_ = new ControlArmRealtime(ip);
}

TestControlArmRT::~TestControlArmRT()
{
    if(control_arm_rt_ != nullptr)
    {
        delete control_arm_rt_;
        control_arm_rt_ = nullptr;
    }
}


void TestControlArmRT::testGetParams()
{
    control_arm_rt_->start();
    sleep(3);

    if(!control_arm_rt_->isConnected())
    {
        printf("URDriverSoftware: control arm disconnected.\n");
        return;
    }

    Joint actual_joint = control_arm_rt_->getActualJoint();
    Joint target_joint = control_arm_rt_->getTargetJoint();
    Joint target_toruqe = control_arm_rt_->getTargetTorque();

    ur_data_type::Joint joint_vel;
    memset(&joint_vel, 0, sizeof(joint_vel));

    // joint_vel.jVal[4] = 0.1;
    // joint_vel.jVal[5] = 0.1;

    for (int i = 0; i != 1000; ++i)
    {
	    control_arm_rt_->speedJointMove(joint_vel, 100.0);
        usleep(8000);
    }

    control_arm_rt_->halt();
}

bool TestControlArmRT::testSendScript()
{
    control_arm_rt_->start();
    sleep(3);

    if(!control_arm_rt_->isConnected())
    {
        printf("URDriverSoftware: control arm disconnected.\n");
        return false;
    }

    vector<Joint> joint_path_end2start;
    Joint position;
    position.jVal[0] = 1.168850; position.jVal[1] = -1.526010; position.jVal[2] = 1.457690; position.jVal[3] = -1.549680; position.jVal[4] = -1.595720; position.jVal[5] = 0.600854;
    joint_path_end2start.push_back(position);
    position.jVal[0] = 1.169710; position.jVal[1] = -1.526020; position.jVal[2] = 1.457740; position.jVal[3] = -1.549620; position.jVal[4] = -1.595530; position.jVal[5] = 0.601501;
    joint_path_end2start.push_back(position);
    position.jVal[0] = 1.191030; position.jVal[1] = -1.528750; position.jVal[2] = 1.467230; position.jVal[3] = -1.555800; position.jVal[4] = -1.596610; position.jVal[5] = 0.622865;
    joint_path_end2start.push_back(position);
    position.jVal[0] = 1.240730; position.jVal[1] = -1.537670; position.jVal[2] = 1.478460; position.jVal[3] = -1.557190; position.jVal[4] = -1.598940; position.jVal[5] = 0.672387;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.300520; position.jVal[1] = -1.544070; position.jVal[2] = 1.487020; position.jVal[3] = -1.557140; position.jVal[4] = -1.601390; position.jVal[5] = 0.732043;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.366400; position.jVal[1] = -1.543170; position.jVal[2] = 1.489800; position.jVal[3] = -1.558860; position.jVal[4] = -1.604300; position.jVal[5] = 0.798013;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.443400; position.jVal[1] = -1.539080; position.jVal[2] = 1.489010; position.jVal[3] = -1.559540; position.jVal[4] = -1.607390; position.jVal[5] = 0.875212;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.515180; position.jVal[1] = -1.528150; position.jVal[2] = 1.481900; position.jVal[3] = -1.560570; position.jVal[4] = -1.610210; position.jVal[5] = 0.946810;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.587730; position.jVal[1] = -1.510490; position.jVal[2] = 1.466900; position.jVal[3] = -1.560660; position.jVal[4] = -1.612650; position.jVal[5] = 1.019360;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615420; position.jVal[1] = -1.502070; position.jVal[2] = 1.458460; position.jVal[3] = -1.559150; position.jVal[4] = -1.613600; position.jVal[5] = 1.047170;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615640; position.jVal[1] = -1.502050; position.jVal[2] = 1.458440; position.jVal[3] = -1.559220; position.jVal[4] = -1.613740; position.jVal[5] = 1.047440;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615760; position.jVal[1] = -1.502050; position.jVal[2] = 1.458460; position.jVal[3] = -1.559170; position.jVal[4] = -1.613660; position.jVal[5] = 1.047630;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615800; position.jVal[1] = -1.502080; position.jVal[2] = 1.458470; position.jVal[3] = -1.559170; position.jVal[4] = -1.613650; position.jVal[5] = 1.047660;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615800; position.jVal[1] = -1.502080; position.jVal[2] = 1.458470; position.jVal[3] = -1.559170; position.jVal[4] = -1.613650; position.jVal[5] = 1.047660;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615760; position.jVal[1] = -1.502050; position.jVal[2] = 1.458460; position.jVal[3] = -1.559170; position.jVal[4] = -1.613660; position.jVal[5] = 1.047630;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615640; position.jVal[1] = -1.502050; position.jVal[2] = 1.458440; position.jVal[3] = -1.559220; position.jVal[4] = -1.613740; position.jVal[5] = 1.047440;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.615420; position.jVal[1] = -1.502070; position.jVal[2] = 1.458460; position.jVal[3] = -1.559150; position.jVal[4] = -1.613600; position.jVal[5] = 1.047170;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.587730; position.jVal[1] = -1.510490; position.jVal[2] = 1.466900; position.jVal[3] = -1.560660; position.jVal[4] = -1.612650; position.jVal[5] = 1.019360;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.515180; position.jVal[1] = -1.528150; position.jVal[2] = 1.481900; position.jVal[3] = -1.560570; position.jVal[4] = -1.610210; position.jVal[5] = 0.946810;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.443400; position.jVal[1] = -1.539080; position.jVal[2] = 1.489010; position.jVal[3] = -1.559540; position.jVal[4] = -1.607390; position.jVal[5] = 0.875212;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.366400; position.jVal[1] = -1.543170; position.jVal[2] = 1.489800; position.jVal[3] = -1.558860; position.jVal[4] = -1.604300; position.jVal[5] = 0.798013;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.300520; position.jVal[1] = -1.544070; position.jVal[2] = 1.487020; position.jVal[3] = -1.557140; position.jVal[4] = -1.601390; position.jVal[5] = 0.732043;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.240730; position.jVal[1] = -1.537670; position.jVal[2] = 1.478460; position.jVal[3] = -1.557190; position.jVal[4] = -1.598940; position.jVal[5] = 0.672387;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.191030; position.jVal[1] = -1.528750; position.jVal[2] = 1.467230; position.jVal[3] = -1.555800; position.jVal[4] = -1.596610; position.jVal[5] = 0.622865;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.169710; position.jVal[1] = -1.526020; position.jVal[2] = 1.457740; position.jVal[3] = -1.549620; position.jVal[4] = -1.595530; position.jVal[5] = 0.601501;
    // joint_path_end2start.push_back(position);
    // position.jVal[0] = 1.168850; position.jVal[1] = -1.526010; position.jVal[2] = 1.457690; position.jVal[3] = -1.549680; position.jVal[4] = -1.595720; position.jVal[5] = 0.600854;
    // joint_path_end2start.push_back(position);

	string script;
	control_arm_rt_->generateServoJScript(joint_path_end2start, 0.0005, 0.33, script);
	if (!control_arm_rt_->sendScript(script))
	{
		printf("TrajRecurrent: sendScript failed\n");
		return false;
	}

    usleep(10000000);

    control_arm_rt_->halt();
    return true;
}