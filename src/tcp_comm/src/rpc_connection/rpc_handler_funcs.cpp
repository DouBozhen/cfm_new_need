#include "rpc_connection.h"
#include <array>
using namespace ForceMasterProto;

void RpcConnection::handleErrorFunc(string err, MessageType cmd_type)
{
	// process msg body
	ResponseCommon response_common;
	response_common.set_is_successful(false);
	response_common.set_error_str(err.c_str());
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(cmd_type, response_data_len);
	printResponseStr(static_cast<uint32_t>(cmd_type), response_common.error_str());

	response_common.release_error_str();
}

void RpcConnection::handlePoweroff(int data_len)
{
	bool result = processPoweroff();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Poweroff, response_data_len);
	printResponseStr(static_cast<uint32_t>(Poweroff), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleStartRobot(int data_len)
{
	bool result = processStartRobot();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(StartRobot, response_data_len);
	printResponseStr(static_cast<uint32_t>(StartRobot), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleCloseRobot(int data_len)
{
	bool result = processCloseRobot();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(CloseRobot, response_data_len);
	printResponseStr(static_cast<uint32_t>(CloseRobot), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTractionForce_SetLevel(int data_len)
{
	Level level;

	if (!level.ParseFromArray(&recv_buffer_[HEADER_LEN], data_len))
	{
		cerr << "Request：TractionForce_SetLevel: parse data failed!" << endl;
	}

	bool result = processTractionForce_SetLevel(level.level());

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TractionForce_SetLevel, response_data_len);
	printResponseStr(static_cast<uint32_t>(TractionForce_SetLevel), response_common.error_str());

	response_common.release_error_str();
}

void RpcConnection::handleTractionForce_GetLevel(int data_len)
{
	int level = 0;
	bool result = processTractionForce_GetLevel(level);

	/* process msg body */
	ResponseLevel response_level;
	response_level.set_level(level);

	ResponseCommon header = response_level.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_level.set_allocated_header(&header);

	uint16_t response_data_len = response_level.ByteSize();
	response_level.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TractionForce_GetLevel, response_data_len);
	printResponseStr(static_cast<uint32_t>(TractionForce_GetLevel), header.error_str());

	header.release_error_str();
	response_level.release_header();
}

void RpcConnection::handleTrajectoryReconstruction_Open(int data_len)
{
	bool result = processTrajectoryReconstruction_Open();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_Open, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_Open), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTrajectoryReconstruction_Close(int data_len)
{
	bool result = processTrajectoryReconstruction_Close();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_Close, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_Close), response_common.error_str());

	response_common.release_error_str();
}

void RpcConnection::handleTrajectoryReconstruction_SetInfo(int data_len)
{
	TrajectoryReconstructionInfo info;
	if (!info.ParseFromArray(&recv_buffer_[HEADER_LEN], data_len))
	{
		cerr << "Request: TrajectoryReconstruction_SetInfo: parse data failed!" << endl;
	}
	else
	{
		cout << "Request: TrajectoryReconstruction_SetInfo: repeated_count = " << info.repeated_count() << ", speed_ratio = " << info.speed_ratio() << endl;
	}

	bool result = processTrajectoryReconstruction_SetInfo(info.repeated_count(), info.speed_ratio());

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_SetInfo, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_SetInfo), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTrajectoryReconstruction_GetInfo(int data_len)
{
	int repeat_count = 0, speed_ratio = 0;
	bool result = processTrajectoryReconstruction_GetInfo(repeat_count, speed_ratio);

	/* process msg body */
	ResponseTrajectoryReconstructionInfo response_info;
	TrajectoryReconstructionInfo info = response_info.info();
	info.set_repeated_count(repeat_count);
	info.set_speed_ratio(speed_ratio);
	response_info.set_allocated_info(&info);

	ResponseCommon header = response_info.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_info.set_allocated_header(&header);

	uint16_t response_data_len = response_info.ByteSize();
	response_info.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_GetInfo, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_GetInfo), header.error_str());

	header.release_error_str();
	response_info.release_header();
	response_info.release_info();
}

void RpcConnection::handleTrajectoryReconstruction_Start(int data_len)
{
	bool result = processTrajectoryReconstruction_Start();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_Start, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_Start), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTrajectoryReconstruction_End(int data_len)
{
	bool result = processTrajectoryReconstruction_End();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_End, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_End), response_common.error_str());

	response_common.release_error_str();
}

void RpcConnection::handleHeartBeat(int data_len)
{
	// cout << "Request: HeartBeat" << endl;
	/* process cmd */
	/* process msg body */
	//printf("-------------- handler HeartBeat --------------\n");
	ResponseCommon response_common;
	response_common.set_is_successful(true);
	response_common.set_error_str(processResultStr(true));
	uint16_t response_data_len = response_common.ByteSize();
	//printf("------------- response_data_len = %d --------------\n", response_data_len);
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(HeartBeat, response_data_len);
	// printResponseStr(static_cast<uint32_t>(HeartBeat), response_common.error_str());

	response_common.release_error_str();
}


void RpcConnection::handleTrajectoryReconstruction_SetSpeedRatio(int data_len)
{
	Level level;

	if (!level.ParseFromArray(&recv_buffer_[HEADER_LEN], data_len))
	{
		cerr << "Request：TractionForce_SetLevel: parse data failed!" << endl;
	}

	bool result = processTrajectoryReconstruction_SetSpeedRatio(level.level());

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_SetSpeedRatio, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_SetSpeedRatio), response_common.error_str());

	response_common.release_error_str();
}
void RpcConnection::handleTrajectoryReconstruction_GetSpeedRatio(int data_len)
{
	int level = 0;
	bool result = processTrajectoryReconstruction_GetSpeedRatio(level);

	/* process msg body */
	ResponseLevel response_level;
	response_level.set_level(level);

	ResponseCommon header = response_level.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_level.set_allocated_header(&header);

	uint16_t response_data_len = response_level.ByteSize();
	response_level.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_GetSpeedRatio, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_GetSpeedRatio), header.error_str());

	header.release_error_str();
	response_level.release_header();
}
void RpcConnection::handleTrajectoryReconstruction_SetRepeatCount(int data_len)
{
	Level level;

	if (!level.ParseFromArray(&recv_buffer_[HEADER_LEN], data_len))
	{
		cerr << "Request：TractionForce_SetLevel: parse data failed!" << endl;
	}

	bool result = processTrajectoryReconstruction_SetRepeatCount(level.level());

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_SetRepeatCount, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_SetRepeatCount), response_common.error_str());

	response_common.release_error_str();
}
void RpcConnection::handleTrajectoryReconstruction_GetRepeatCount(int data_len)
{
	int level = 0;
	bool result = processTrajectoryReconstruction_GetRepeatCount(level);

	/* process msg body */
	ResponseLevel response_level;
	response_level.set_level(level);

	ResponseCommon header = response_level.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_level.set_allocated_header(&header);

	uint16_t response_data_len = response_level.ByteSize();
	response_level.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(TrajectoryReconstruction_GetRepeatCount, response_data_len);
	printResponseStr(static_cast<uint32_t>(TrajectoryReconstruction_GetRepeatCount), header.error_str());

	header.release_error_str();
	response_level.release_header();
}

void RpcConnection::handleGetSensorStatus(int data_len)
{
	// cout << "Request: GetSensorStatus" << endl;
	bool status = true;
	bool result = true;

	/* process msg body */
	ResponseSensorStatus response_sensor_status;
	response_sensor_status.set_status(status);

	ResponseCommon header = response_sensor_status.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_sensor_status.set_allocated_header(&header);

	uint16_t response_data_len = response_sensor_status.ByteSize();
	response_sensor_status.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(GetSensorStatus, response_data_len);
	// printResponseStr(static_cast<uint32_t>(GetSensorStatus), header.error_str());

	header.release_error_str();
	response_sensor_status.release_header();
}

void RpcConnection::handleImpedanceMotion_Open(int data_len)
{
	bool result = true;
	/* process msg body */

	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(ImpedanceMotion_Open, response_data_len);
	printResponseStr(static_cast<uint32_t>(ImpedanceMotion_Open), response_common.error_str());

	response_common.release_error_str();
}

void RpcConnection::handleImpedanceMotion_Close(int data_len)
{
	bool result = true;

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(ImpedanceMotion_Close, response_data_len);
	printResponseStr(static_cast<uint32_t>(ImpedanceMotion_Close), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleImpedanceMotion_SetRangelevel(int data_len)
{
	Level level;
	if (!level.ParseFromArray(&recv_buffer_[HEADER_LEN], data_len))
	{
		cerr << "Request: ImpedanceMotion_SetRangelevel: parse data failed!" << endl;
	}
	else
	{
		cout << "Request: ImpedanceMotion_SetRangelevel: " << level.level() << endl;
	}

	bool result = true; //processImpedanceMotion_SetRangelevel(level.level());

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(ImpedanceMotion_SetRangelevel, response_data_len);
	printResponseStr(static_cast<uint32_t>(ImpedanceMotion_SetRangelevel), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleImpedanceMotion_GetRangelevel(int data_len)
{
	int level = 0;
	bool result = true; //processImpedanceMotion_GetRangelevel(level);

	/* process msg body */
	ResponseLevel response_level;
	response_level.set_level(level);

	ResponseCommon header = response_level.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_level.set_allocated_header(&header);

	uint16_t response_data_len = response_level.ByteSize();
	response_level.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(ImpedanceMotion_GetRangelevel, response_data_len);
	printResponseStr(static_cast<uint32_t>(ImpedanceMotion_GetRangelevel), header.error_str());

	header.release_error_str();
	response_level.release_header();
}

void RpcConnection::handleGetJointPosition(int data_len)
{
	bool result = true;
	std::array<double, 6> server_position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	// server_position = processGetJointPosition();

	ResponseJointPosition response_joint;
	JointPosition position = response_joint.position();

	int i = 0;
	for (auto it = server_position.begin(); it != server_position.end(); ++it)
	{
		position.add_joint(*it);
		// cout << "joint[" << i << "] = " << *it << endl;
		i++;
	}
	position.set_joint_num(6);

	/* process msg body */
	response_joint.set_allocated_position(&position);
	ResponseCommon header = response_joint.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response_joint.set_allocated_header(&header);

	uint16_t response_data_len = response_joint.ByteSize();
	response_joint.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(GetJointPosition, response_data_len);
	printResponseStr(static_cast<uint32_t>(GetJointPosition), header.error_str());

	header.release_error_str();
	response_joint.release_header();
	response_joint.release_position();
}

void RpcConnection::handleGetCartPosition(int data_len)
{
	std::array<double, 6> server_position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	// server_position = processGetCartPosition();

	CartPosition position;
	position.set_x(server_position[0]);
	position.set_y(server_position[1]);
	position.set_z(server_position[2]);
	position.set_a(server_position[3]);
	position.set_b(server_position[4]);
	position.set_c(server_position[5]);

	for (int i = 0; i != server_position.size(); ++i)
	{
		//printf("get cart_position[%d] = %f, ", i, server_position[i]);
	}
	//printf("\n");

	//cout << "cart position: " << position.x() << ", " << position.y() << ", " << position.z() << ", " << position.a() << ", " << position.b() << ", " << position.c() << endl;

	/* process msg body */
	ResponseCartPosition response_cart;
	response_cart.set_allocated_position(&position);
	ResponseCommon header = response_cart.header();
	header.set_is_successful(true);
	header.set_error_str(processResultStr(true));
	response_cart.set_allocated_header(&header);

	uint16_t response_data_len = response_cart.ByteSize();
	response_cart.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(GetCartPosition, response_data_len);
	// printResponseStr(static_cast<uint32_t>(GetCartPosition), header.error_str());

	header.release_error_str();
	response_cart.release_header();
	response_cart.release_position();
}

void RpcConnection::handleSetCartPosition(int data_len)
{
	CartPosition position;
	bool result = false;
	if (!position.ParseFromArray(&recv_buffer_[HEADER_LEN], data_len))
	{
		cerr << "Request SetCartPosition: parse data failed!" << endl;
	}
	else
	{
		// cout << "Request SetCartPosition: " << position.x() << ", " << position.y() << ", " << position.z() << ", " << position.a() << ", " << position.b() << ", " << position.c() << endl;
	}
	std::array<double, 6> server_position = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
	server_position[0] = position.x();
	server_position[1] = position.y();
	server_position[2] = position.z();
	server_position[3] = position.a();
	server_position[4] = position.b();
	server_position[5] = position.c();

	result = true; //processSetCartPosition(server_position);

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(SetCartPosition, response_data_len);
	printResponseStr(static_cast<uint32_t>(SetCartPosition), response_common.error_str());

	response_common.release_error_str();
}

void RpcConnection::handleRemotePulling_Open(int data_len)
{
	bool result = true; //processRemotePulling_Open();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(RemotePulling_Open, response_data_len);
	printResponseStr(static_cast<uint32_t>(RemotePulling_Open), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleRemotePulling_Close(int data_len)
{
	bool result = true; // processRemotePulling_Close();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(RemotePulling_Close, response_data_len);
	printResponseStr(static_cast<uint32_t>(RemotePulling_Close), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleOmegaPulling_Open(int data_len)
{
	bool result = true; //processOmegaPulling_Open();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(OmegaPulling_Open, response_data_len);
	printResponseStr(static_cast<uint32_t>(OmegaPulling_Open), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleOmegaPulling_Close(int data_len)
{
	bool result = true; //processOmegaPulling_Close();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(OmegaPulling_Close, response_data_len);
	printResponseStr(static_cast<uint32_t>(OmegaPulling_Close), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_UpdateStartPose(int data_len)
{
	bool result = processTraining_UpdateStartPose();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_UpdateStartPose, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_UpdateStartPose), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_ReturnStartPose(int data_len)
{
	bool result = processTraining_ReturnStartPose();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_ReturnStartPose, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_ReturnStartPose), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_GenerateTrajProximal(int data_len)
{
	bool result = processTraining_GenerateTrajProximal();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_GenerateTrajProximal, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_GenerateTrajProximal), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_GenerateTrajDistal(int data_len)
{
	bool result = processTraining_GenerateTrajDistal();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_GenerateTrajDistal, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_GenerateTrajDistal), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_UpdateEndPoseProximal(int data_len)
{
	bool result = processTraining_UpdateEndPoseProximal();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_UpdateEndPoseProximal, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_UpdateEndPoseProximal), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_UpdateEndPoseDistal(int data_len)
{
	bool result = processTraining_UpdateEndPoseDistal();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_UpdateEndPoseDistal, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_UpdateEndPoseDistal), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_StartTrajRecurr(int data_len)
{
	bool result = processTraining_StartTrajRecurr();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_StartTrajRecurr, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_StartTrajRecurr), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_EndTrajRecurr(int data_len)
{
	bool result = processTraining_EndTrajRecurr();

	/* process msg body */
	ResponseCommon response_common;
	response_common.set_is_successful(result);
	response_common.set_error_str(processResultStr(result));
	uint16_t response_data_len = response_common.ByteSize();
	response_common.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_EndTrajRecurr, response_data_len);
	printResponseStr(static_cast<uint32_t>(Training_EndTrajRecurr), response_common.error_str());
	response_common.release_error_str();
}

void RpcConnection::handleTraining_IsReachedStartPose(int data_len)
{
	bool status = true;
	bool result = processTraining_IsReachedStartPose(status);

	/* process msg body */
	ResponseStatus response;
	response.set_status(status);

	ResponseCommon header = response.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response.set_allocated_header(&header);

	uint16_t response_data_len = response.ByteSize();
	response.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_IsReachedStartPose, response_data_len);
	// printResponseStr(static_cast<uint32_t>(Training_IsGenerateTrajComplete), header.error_str());

	header.release_error_str();
	response.release_header();
}

void RpcConnection::handleTraining_IsReturnStartPose(int data_len)
{
	bool status = true;
	bool result = processTraining_IsReturnStartPose(status);
	printf("------------------return start position %d-----------------\n", result);

	/* process msg body */
	ResponseStatus response;
	response.set_status(status);

	ResponseCommon header = response.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response.set_allocated_header(&header);

	uint16_t response_data_len = response.ByteSize();
	response.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_IsReturnStartPose, response_data_len);
	// printResponseStr(static_cast<uint32_t>(Training_IsGenerateTrajComplete), header.error_str());

	header.release_error_str();
	response.release_header();
}

void RpcConnection::handleTraining_IsGenerateProximalTrajComplete(int data_len)
{
	bool status = true;
	bool result = processTraining_IsGenerateProximalTrajComplete(status);

	/* process msg body */
	ResponseStatus response;
	response.set_status(status);

	ResponseCommon header = response.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response.set_allocated_header(&header);

	uint16_t response_data_len = response.ByteSize();
	response.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_IsGenerateProximalTrajComplete, response_data_len);
	// printResponseStr(static_cast<uint32_t>(Training_IsGenerateTrajComplete), header.error_str());

	header.release_error_str();
	response.release_header();
}

void RpcConnection::handleTraining_IsGenerateDistalTrajComplete(int data_len)
{
	bool status = true;
	bool result = processTraining_IsGenerateDistalTrajComplete(status);

	/* process msg body */
	ResponseStatus response;
	response.set_status(status);

	ResponseCommon header = response.header();
	header.set_is_successful(result);
	header.set_error_str(processResultStr(result));
	response.set_allocated_header(&header);

	uint16_t response_data_len = response.ByteSize();
	response.SerializeToArray(&send_buffer_[HEADER_LEN], response_data_len);

	setSendBuffferSize(HEADER_LEN + response_data_len);
	headerToBuffer(Training_IsGenerateDistalTrajComplete, response_data_len);
	// printResponseStr(static_cast<uint32_t>(Training_IsGenerateTrajComplete), header.error_str());

	header.release_error_str();
	response.release_header();
}
