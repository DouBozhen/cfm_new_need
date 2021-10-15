
#include "arm_state_rt.h"

using namespace std;
using namespace ur_data_type;

double ArmStateRealtime::ntohd(uint64_t data) 
{
	double x;
	data = be64toh(data);
	x = *(double *)&data;
	return x;
}

void ArmStateRealtime::decode(uint8_t* buf)
{
	int offset = sizeof(int);
    std::array<double, 6> joint = {};
    std::array<double, 6> pose = {};
    std::array<double, 3> cart = {};
	uint64_t unpack_to = 0;

	unpack_to = *(uint64_t*)&buf[offset]; // 4-11: time
	time_ = ntohd(unpack_to);
	offset += sizeof(uint64_t);

	decodeVector(buf, offset, 6, joint); // 12-59： target joint
    for(int j = 0; j != 6; ++j)
    {
        joint_position_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 60-107：joint_vel_
    for(int j = 0; j != 6; ++j)
    {
        joint_vel_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 108-155: joint_acc_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        joint_acc_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 156-203: joint_current_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        joint_current_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 104-255: joint_torque_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        joint_torque_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 256-299: actual_joint_position_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        actual_joint_position_.jVal[j] = joint[j];
    }
    offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 300-347: actual_joint_vel_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        actual_joint_vel_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

	decodeVector(buf, offset, 6, joint); // 348-395: actual_current_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        actual_current_.jVal[j] = joint[j];
    }
	offset += sizeof(double) * 6;

    if(protocol_version_ < 1.9)
    {
        decodeVector(buf, offset, 3, cart); // 396-420: tool_point_acc_
        tool_point_acc_.x = cart[0];
        tool_point_acc_.y = cart[1];
        tool_point_acc_.z = cart[2];
        offset += sizeof(double) * 3;
        offset += sizeof(double) * 15; // 420-539: null

        decodeVector(buf, offset, 6, pose); //540-587: tcp_force_
        tcp_force_.point.x = pose[0];
        tcp_force_.point.y = pose[1];
        tcp_force_.point.z = pose[2];
        tcp_force_.rpy.rx = pose[3];
        tcp_force_.rpy.ry = pose[4];
        tcp_force_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); // 588-635: actual_tool_pose_
        actual_tool_pose_.point.x = pose[0];
        actual_tool_pose_.point.y = pose[1];
        actual_tool_pose_.point.z = pose[2];
        actual_tool_pose_.rpy.rx = pose[3];
        actual_tool_pose_.rpy.ry = pose[4];
        actual_tool_pose_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); // 636-683:actual_tcp_vel_
        actual_tcp_vel_.point.x = pose[0];
        actual_tcp_vel_.point.y = pose[1];
        actual_tcp_vel_.point.z = pose[2];
        actual_tcp_vel_.rpy.rx = pose[3];
        actual_tcp_vel_.rpy.ry = pose[4];
        actual_tcp_vel_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;
    }
    else 
    {
        decodeVector(buf, offset, 6, joint); // 396-443: control_current_
        for(int j = 0; j != JOINT_NUM; ++j)
        {
            control_current_.jVal[j] = joint[j];
        }
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); // 444-491: actual_tool_pose_
        actual_tool_pose_.point.x = pose[0];
        actual_tool_pose_.point.y = pose[1];
        actual_tool_pose_.point.z = pose[2];
        actual_tool_pose_.rpy.rx = pose[3];
        actual_tool_pose_.rpy.ry = pose[4];
        actual_tool_pose_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); // 492-539: actual_tcp_vel_
        actual_tcp_vel_.point.x = pose[0];
        actual_tcp_vel_.point.y = pose[1];
        actual_tcp_vel_.point.z = pose[2];
        actual_tcp_vel_.rpy.rx = pose[3];
        actual_tcp_vel_.rpy.ry = pose[4];
        actual_tcp_vel_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); //540-587: tcp_force_
        tcp_force_.point.x = pose[0];
        tcp_force_.point.y = pose[1];
        tcp_force_.point.z = pose[2];
        tcp_force_.rpy.rx = pose[3];
        tcp_force_.rpy.ry = pose[4];
        tcp_force_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); //540-587: target_tool_pose_
        target_tool_pose_.point.x = pose[0];
        target_tool_pose_.point.y = pose[1];
        target_tool_pose_.point.z = pose[2];
        target_tool_pose_.rpy.rx = pose[3];
        target_tool_pose_.rpy.ry = pose[4];
        target_tool_pose_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;

        decodeVector(buf, offset, 6, pose); //588-635: target_tcp_vel_
        target_tcp_vel_.point.x = pose[0];
        target_tcp_vel_.point.y = pose[1];
        target_tcp_vel_.point.z = pose[2];
        target_tcp_vel_.rpy.rx = pose[3];
        target_tcp_vel_.rpy.ry = pose[4];
        target_tcp_vel_.rpy.rz = pose[5];
        offset += sizeof(double) * 6;
    }

	digital_input_ = *(uint64_t *)&buf[offset]; // 684-691: digital_input_
	digital_input_ = ntohd(digital_input_);
	//printf("decode digital inout = 0x%08x\n", digital_input_);
	offset += sizeof(uint64_t);

	decodeVector(buf, offset, 6, joint); // 692-739: motor_temperatures_
    for(int j = 0; j != JOINT_NUM; ++j)
    {
        motor_temperatures_.jVal[j] = joint[j];
    }
    offset += sizeof(double) * 6;

    unpack_to= *(uint64_t*)&buf[offset]; // 740-747: controller_timer_
	controller_timer_ = ntohd(unpack_to);

    if (protocol_version_ > 1.6)
    {
        offset += sizeof(double) * 2;
		unpack_to = *(uint64_t*)&buf[offset]; // 764-771: robot_mode_
		robot_mode_ = ntohd(unpack_to);

		if (protocol_version_ > 1.7) 
        {
			offset += sizeof(double); 
			decodeVector(buf, offset, 6, joint); // 780-827: joint_control_modes_
            for(int j = 0; j != JOINT_NUM; ++j)
            {
                joint_control_modes_.jVal[j] = joint[j];
            }
		}
	}

    if (protocol_version_ > 1.8)
    {
        offset += sizeof(double) * 6;

		unpack_to = *(uint64_t *)&buf[offset]; // 828-835: safety_mode_
		safety_mode_ = ntohd(unpack_to);
        offset += sizeof(uint64_t);

        offset += sizeof(double) * 6;

        offset += sizeof(double) * 3; // tool acc value

        offset += sizeof(double) * 6;

		unpack_to = *(uint64_t *)&buf[offset]; // 955-963: vel_scaling_
        vel_scaling_ = ntohd(unpack_to);
		offset += sizeof(double);

		unpack_to = *(uint64_t *)&buf[offset];
        linear_momentum_norm_ = ntohd(unpack_to); // 964-971: linear_momentum_norm_
        offset += sizeof(double) * 1;

        offset += sizeof(double) * 2;

		unpack_to = *(uint64_t*)&buf[offset]; // 987-995: main_voltage_
        main_voltage_ = ntohd(unpack_to);
		offset += sizeof(double);

		unpack_to = *(uint64_t *)&buf[offset]; // 996-1003: robot_voltage_
		robot_voltage_ = ntohd(unpack_to);
		offset += sizeof(double);

		unpack_to = *(uint64_t*)&buf[offset]; // 1003-1011: robot_current_
		robot_current_ = ntohd(unpack_to);
        offset += sizeof(double);
    
        decodeVector(buf, offset, 6, joint); // 1012-1059: joint_voltage_
        for(int j = 0; j != JOINT_NUM; ++j)
        {
            joint_voltage_.jVal[j] = joint[j];
        }
        offset += sizeof(double) * 6;

		unpack_to = *(uint64_t*)&buf[offset]; // digital_outputs_
        double digital_outputs = ntohd(unpack_to);
		digital_outputs_ = static_cast<uint64_t>(digital_outputs);
        offset += sizeof(uint64_t);

		unpack_to = *(double*)&buf[offset]; // program_state_
		program_state_ = ntohd(unpack_to);
        offset += sizeof(double);
	}
}
