#ifndef RPC_CONNECTION_H
#define RPC_CONNECTION_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <mutex>

#include "connection.h"
#include "ForceMaster.pb.h"

using namespace std;

class RpcConnection : public Connection
{
public:
    RpcConnection(int socket, fd_set &server_fd_set);
    ~RpcConnection();

	virtual bool startRunning() override;
	virtual void disConning() override;

	static void* recvMessageThread(void* param);
	static void* sendMessageThread(void* param);

	virtual bool isConning() override;
	virtual bool isExit() override;
	virtual int getConnSocket() override;

private:
	enum {
		DATA_BUFFER_LEN = 1024, HEADER_LEN = 6, CMD_TYPE_LEN = 4
	};

    int s_;
	int conn_fd_[CONCURRENT_MAX];
	int* conn_fds_ptr_;
	int index_;
	fd_set* server_fd_set_ptr_;

    bool is_received_;

    char* send_buffer_;
	char* recv_buffer_;
	int recv_buffer_size_;
	int send_buffer_size_;
    int recv_times_;
    int send_times_;

    pthread_mutex_t mutex_;

    bool is_conning_;
	bool is_exit_;

    pthread_t send_thread_;
    pthread_t recv_thread_;   

    bool is_concordance_;

    typedef void (RpcConnection::*HandleFuncPtr)(int);
	typedef struct
	{
		ForceMasterProto::MessageType cmd_type;
		HandleFuncPtr request_func_ptr;
	}RpcService;

	typedef struct
	{
		ForceMasterProto::MessageType cmd_type;
		void* request_data_ptr;
	}Request;

	std::vector<RpcService> rpc_table_;
	pthread_mutex_t rpc_table_mutex_;

	void initRpcTable();
	HandleFuncPtr getHandlerByCmdType(ForceMasterProto::MessageType cmd_type);

	void handleErrorFunc(string err, ForceMasterProto::MessageType cmd_type);
	void handlePoweroff(int data_len);
	void handleStartRobot(int data_len);
	void handleCloseRobot(int data_len);
	void handleTractionForce_SetLevel(int data_len);
	void handleTractionForce_GetLevel(int data_len);
	void handleTrajectoryReconstruction_Open(int data_len);
	void handleTrajectoryReconstruction_Close(int data_len);
	void handleTrajectoryReconstruction_SetInfo(int data_len);
	void handleTrajectoryReconstruction_GetInfo(int data_len);
	void handleTrajectoryReconstruction_Start(int data_len);
	void handleTrajectoryReconstruction_End(int data_len);
	void handleHeartBeat(int data_len);
	void handleTrajectoryReconstruction_SetSpeedRatio(int data_len);
	void handleTrajectoryReconstruction_GetSpeedRatio(int data_len);
	void handleTrajectoryReconstruction_SetRepeatCount(int data_len);
	void handleTrajectoryReconstruction_GetRepeatCount(int data_len);

	/* new needs */
	void handleTraining_UpdateStartPose(int data_len);
	void handleTraining_ReturnStartPose(int data_len);
	void handleTraining_GenerateTrajProximal(int data_len);
	void handleTraining_GenerateTrajDistal(int data_len);
	void handleTraining_UpdateEndPoseProximal(int data_len);
	void handleTraining_UpdateEndPoseDistal(int data_len);
	void handleTraining_StartTrajRecurr(int data_len);
	void handleTraining_EndTrajRecurr(int data_len);

	void handleTraining_IsReachedStartPose(int data_len);
	void handleTraining_IsReturnStartPose(int data_len);
	void handleTraining_IsGenerateProximalTrajComplete(int data_len);
	void handleTraining_IsGenerateDistalTrajComplete(int data_len);

	/* useless */
	void handleGetSensorStatus(int data_len);
	void handleImpedanceMotion_Open(int data_len);
	void handleImpedanceMotion_Close(int data_len);
	void handleImpedanceMotion_SetRangelevel(int data_len);
	void handleImpedanceMotion_GetRangelevel(int data_len);
	void handleGetJointPosition(int data_len);
	void handleGetCartPosition(int data_len);
	void handleSetCartPosition(int data_len);
	void handleRemotePulling_Open(int data_len);
	void handleRemotePulling_Close(int data_len);
	void handleOmegaPulling_Open(int data_len);
	void handleOmegaPulling_Close(int data_len);

	uint16_t bigToLittile16(uint16_t data);
	uint32_t bigToLittile32(uint32_t data);

	uint16_t littileToBig16(uint16_t data);
	uint32_t littileToBig32(uint32_t data);

	uint32_t convertRequestCmdType(ForceMasterProto::MessageType cmd_type);
	uint32_t convertResponseCmdType(ForceMasterProto::MessageType cmd_type);
	uint16_t convertRequestDataLen(uint16_t data_len);
	uint16_t convertResponseDataLen(uint16_t data_len);

	void setSendBuffferSize(int size);
	void cmdTypeToBuffer(const uint32_t &cmd_type);
	void dataLenToBuffer(const uint16_t& len);
	void headerToBuffer(ForceMasterProto::MessageType cmd_type, uint16_t data_len);

	void getTime();
	void printResponseStr(uint32_t cmd_type, std::string data);
	std::string processResultStr(bool is_success);

	bool processPoweroff();
	bool processStartRobot();
	bool processCloseRobot();
	bool processTractionForce_SetLevel(int level);
	bool processTractionForce_GetLevel(int &level);
	bool processTrajectoryReconstruction_Open();
	bool processTrajectoryReconstruction_Close();
	bool processTrajectoryReconstruction_SetInfo(int repeat_count, int speed_ratio);
	bool processTrajectoryReconstruction_GetInfo(int &repeat_count, int &speed_ratio);
	bool processTrajectoryReconstruction_Start();
	bool processTrajectoryReconstruction_End();
	bool processHeartBeat();
	bool processTrajectoryReconstruction_SetSpeedRatio(int ratio);
	bool processTrajectoryReconstruction_GetSpeedRatio(int &ratio);
	bool processTrajectoryReconstruction_SetRepeatCount(int count);
	bool processTrajectoryReconstruction_GetRepeatCount(int &count);
	
	/* new needss */ 
	bool processTraining_UpdateStartPose();
	bool processTraining_ReturnStartPose();
	bool processTraining_GenerateTrajProximal();
	bool processTraining_GenerateTrajDistal();
	bool processTraining_UpdateEndPoseProximal();
	bool processTraining_UpdateEndPoseDistal();
	bool processTraining_StartTrajRecurr();
	bool processTraining_EndTrajRecurr();

	bool processTraining_IsReachedStartPose(bool status);
	bool processTraining_IsReturnStartPose(bool status);
	bool processTraining_IsGenerateProximalTrajComplete(bool status);
	bool processTraining_IsGenerateDistalTrajComplete(bool status);

	/* useless */
	bool processGetSensorStatus();
	bool processImpedanceMotion_Open();
	bool processImpedanceMotion_Close();
	bool processImpedanceMotion_SetRangelevel();
	bool processImpedanceMotion_GetRangelevel();
	bool processGetJointPosition();
	bool processGetCartPosition();
	bool processSetCartPosition();
	bool processRemotePulling_Open();
	bool processRemotePulling_Close();
	bool processOmegaPulling_Open();
	bool processOmegaPulling_Close();

	RpcConnection();
};

#endif

