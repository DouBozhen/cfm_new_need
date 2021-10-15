#include <sys/time.h>
#include <unistd.h>
#include <string>
#include <fstream>
#include <stdio.h>

#include "sleep_milli.h"
#include "arm_comm_rt.h"

using namespace std;

bool ArmCommRealtime::setPayload(double pay_load)
{
	if (PAY_LOAD_MIN < pay_load && pay_load < PAY_LOAD_MAX)
    {
		char cmd[256];
		sprintf(cmd, "sec setOut():\n\tset_payload(%1.3f)\nend\n", pay_load);

        return sendMessage(std::string(cmd));
	}
    
    printf("ArmCommRealtime: Payload value %f out of range[%f, %f].\n",
        pay_load, PAY_LOAD_MIN, PAY_LOAD_MAX);
    return false;
}

bool ArmCommRealtime::speedJointMove(ur_data_type::Joint joint_vel, double acc)
{
	char cmd[1024];
	sprintf(cmd, "speedj([%1.5f, %1.5f, %1.5f, %1.5f, %1.5f, %1.5f], %f, 0.02)\n", 
        joint_vel.jVal[0],
        joint_vel.jVal[1], 
        joint_vel.jVal[2], 
        joint_vel.jVal[3], 
        joint_vel.jVal[4], 
        joint_vel.jVal[5], 
        acc
    );

    if(sendMessage(std::string(cmd)))
    {
        if ( joint_vel.jVal[0] != 0. 
            || joint_vel.jVal[1] != 0. 
            || joint_vel.jVal[2] != 0. 
            || joint_vel.jVal[3] != 0. 
            || joint_vel.jVal[4] != 0. 
            || joint_vel.jVal[5] != 0.) 
        {
            //If a joint speed is set, make sure we stop it again after some time if the user doesn't
            safety_count_ = 0; 
        }

        return true;
    }
    return false;
}

bool ArmCommRealtime::jointMove(ur_data_type::Joint joint_position, 
    double acc, double vel, double during, double r)
{
    char cmd[1024];
	sprintf(cmd, "movej([%1.5f,%1.5f,%1.5f,%1.5f,%1.5f,%1.5f],%1.5f,%1.5f,%1.5f,%1.5f)\n",
        joint_position.jVal[0],
        joint_position.jVal[1], 
        joint_position.jVal[2], 
        joint_position.jVal[3], 
        joint_position.jVal[4], 
        joint_position.jVal[5],
        acc,
        vel,
        during,
        r
    );

    std::string string_cmd(cmd);

    printf("string_cmd: %s", string_cmd.c_str());
    return sendMessage(string_cmd);
}

bool ArmCommRealtime::servoJ(ur_data_type::Joint joint_position, 
    double acc, double vel, double t1, double t2, int gain)
{
	char cmd[1024];
	sprintf(cmd, "servoj([%1.5f,%1.5f,%1.5f,%1.5f,%1.5f,%1.5f],%1.5f,%1.5f,%1.5f,%1.5f,%d)\n",
        joint_position.jVal[0],
        joint_position.jVal[1], 
        joint_position.jVal[2], 
        joint_position.jVal[3], 
        joint_position.jVal[4], 
        joint_position.jVal[5],
        acc,
        vel,
        t1,
        t2,
        gain
    );
    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::lineMove(ur_data_type::CartPose target_pose, 
    double acc, double vel, double during, double r)
{
    printf("ArmCommRealtime::lineMove\n");
	//char cmd[1024];
	std::string cmd;
	cmd = "def PathRecodingProg():\n";
	cmd += "  global switch=0\n";
	cmd += "  global speed=0\n";
	cmd += "  global speed_low=0.5\n";
	cmd += "  global speed_high=1.0\n";
	cmd += "  thread Thread_1():\n";
	cmd += "    while (True):\n";
	cmd += "      global ok=socket_open(\"127.0.0.1\", 30003)\n";
	cmd += "      if (switch == 0):\n";
	cmd += "        global speed=speed_low\n";
	cmd += "      else:\n";
	cmd += "        if (switch == 1):\n";
	cmd += "          global speed=speed_high\n";
	cmd += "        end\n";
	cmd += "      end\n";
	cmd += "      socket_send_string(\"set speed\")\n";
	cmd += "      socket_send_string(speed)\n";
	cmd += "      socket_send_byte(10)\n";
	cmd += "      sync()\n";
	cmd += "    end\n";
	cmd += "  end\n";
	cmd += "  threadId_Thread_1 = run Thread_1()\n";
	cmd += "  sleep(0.8)\n";
	cmd += "  movel(p[" 
        + std::to_string(target_pose.point.x) 
        + "," + std::to_string(target_pose.point.y) 
        + "," + std::to_string(target_pose.point.z) 
        + "," + std::to_string(target_pose.rpy.rx) 
        + "," + std::to_string(target_pose.rpy.ry) 
        + "," + std::to_string(target_pose.rpy.rz) 
        + "],a="
        + std::to_string(acc)
        + ",v="
        + std::to_string(vel)
        + ")\n";
	cmd += "  stopl(1.210567178230453)\n";
	cmd += "  global switch=1\n";
	cmd += "  sleep(1.8)\n";
	cmd += "end\n";
    //return true;
    return sendMessage(cmd);
}

bool ArmCommRealtime::stopJointMove()
{
    char cmd[1024];
	sprintf(cmd, "stopj(5)\n");
    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::ArmCommRealtime::setToolVoltage(unsigned int voltage)
{
	char cmd[256];
	sprintf(cmd, "sec setOut():\n\tset_tool_voltage(%d)\nend\n", voltage);
    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::setFlag(unsigned int index, bool status)
{
	char cmd[256];
	sprintf(cmd, "sec setOut():\n\tset_flag(%d, %s)\nend\n", 
        index, 
        status ? "True" : "False"
    );
    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::setDigitalOutput(unsigned int index, bool status)
{
	char cmd[256];
	if (arm_state_rt_->getProtocolVersion() < 2) 
	{
		sprintf(cmd, "sec setOut():\n\tset_digital_out(%d, %s)\nend\n", 
            index,
			status ? "True" : "False"
        );

        return sendMessage(std::string(cmd));
	}
	
    if (index > 9) 
	{
		sprintf(cmd,
			"sec setOut():\n\tset_configurable_digital_out(%d, %s)\nend\n",
			index - 10, 
            status ? "True" : "False"
        ); 
	}
	else if (index > 7) 
	{
		sprintf(cmd, "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n",
			index - 8, 
            status ? "True" : "False"
        );

	}
	else {
		sprintf(cmd, "sec setOut():\n\tset_standard_digital_out(%d, %s)\nend\n",
			index, 
            status ? "True" : "False"
        );
	}

    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::setAnalogOutput(unsigned int index, double value)
{
	char cmd[256];
	sprintf(cmd, "sec setOut():\n\tset_standard_analog_out(%d, %1.4f)\nend\n", 
        index, 
        value
    );
    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::setToolDigitalOutput(unsigned int index, bool status)
{
	char cmd[256];
	sprintf(cmd, "sec setOut():\n\tset_tool_digital_out(%d, %s)\nend\n",
        index,
        status ? "True" : "False"
    );
    return sendMessage(std::string(cmd));
}

bool ArmCommRealtime::generateServoJScript(std::vector<ur_data_type::Joint> path, 
        double vel_min, double vel_max, std::string &script)
{
    if(path.empty())
    {
        printf("ArmCommRealtime: joint path is empty.\n");
        return false;
    }

	script = "def PathRecodingProg():\n";
	script += "  global switch=0\n";
	script += "  global speed=0\n";
	script += "  global speed_low=" 
        + std::to_string(vel_min) 
        + "\n";
	script += "  global speed_high="
        + std::to_string(vel_max) 
        + "\n";
	script += "  thread Thread_1():\n";
	script += "    while (True):\n";
	script += "      global ok=socket_open(\"127.0.0.1\", 30003)\n";
	script += "      if (switch == 0):\n";
	script += "        global speed=speed_low\n";
	script += "      else:\n";
	script += "        if (switch == 1):\n";
	script += "          global speed="
        + std::to_string((vel_min + vel_max)/2) 
        + "\n";
	script += "        end\n";
	script += "      end\n";
	script += "      socket_send_string(\"set speed\")\n";
	script += "      socket_send_string(speed)\n";
	script += "      socket_send_byte(10)\n";
	script += "      sync()\n";
	script += "    end\n";
	script += "  end\n";
	script += "  threadId_Thread_1 = run Thread_1()\n";
	//script += "  sleep(1)\n"; 
	script += "  sleep(0.1)\n";

    for (int i = 0; i < path.size(); i++)
    {
        std::string joint_position = "[" 
            + std::to_string(path[i].jVal[0]) + "," 
            + std::to_string(path[i].jVal[1]) + "," 
            + std::to_string(path[i].jVal[2]) + "," 
            + std::to_string(path[i].jVal[3]) + "," 
            + std::to_string(path[i].jVal[4]) + "," 
            + std::to_string(path[i].jVal[5]) + "]";
        script += "  servoj(" + joint_position + "," + std::to_string((vel_max + vel_min)/2) + ",0.5,0.08,0.2,100)\n";
    }
	script += "  stopj(1)\n";
	script += "  global switch=1\n";
	script += "  sleep(2)\n";
	script += "end\n";
	// std::cout << script << std::endl;
	return true;
}

bool ArmCommRealtime::sendServoJScript(string script)
{
    return sendMessage(std::string(script));
}

bool ArmCommRealtime::initGlobalSpeed()
{
    std::string script("");
    script = "def setSpeedProg():\n";
	script += "  global switch=0\n";
	script += "  global speed=0\n";
	script += "  global speed_low=0.0\n";
	script += "  global speed_high=1.0\n";
   	script += "  thread Thread_1():\n";
	script += "    while (True):\n";
	script += "      global ok=socket_open(\"127.0.0.1\", 30003)\n";
	script += "      global speed=speed_high\n";
	script += "      socket_send_string(\"set speed\")\n";
	script += "      socket_send_string(speed)\n";
	script += "      socket_send_byte(10)\n";
	script += "      sync()\n";
	script += "    end\n";
	script += "  end\n";
	script += "  threadId_Thread_1 = run Thread_1()\n";
	script += "  sleep(1)\n";
	script += "end\n";

	return sendMessage(std::string(script));
}