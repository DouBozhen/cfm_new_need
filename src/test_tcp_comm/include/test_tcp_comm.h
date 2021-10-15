#ifndef TEST_TCP_COMM
#define TEST_TCP_COMM

#include <fcntl.h>
#include "tcp_client.h"
#include "ForceMaster.pb.h"

#define CLIENT_PORT 30800
#define TCP_COMM_BUF_SIZE 512

class TestTcpComm
{
public: 
    TestTcpComm();
    ~TestTcpComm();
    bool init();
    bool isConnected();

    bool testAllParams();

    bool testPowerOff();
    bool testStopRobot();

    bool testHeartBeat();
    bool testSetForceLevel(int level);
    bool testGetForceLevel();
    bool testSetRepeatedCount(int count);
    bool testGetRepeatedCount();
    bool testSetSpeedRatio(int ratio);
    bool testGetSpeedRatio();

    bool testOpenTrajRecurr();
    bool testCloseTrajRecurr();
    bool testStartTrajRecurr();
    bool testStopTrajRecurr();

    bool trestTrainingUpdateStartPose();
    bool trestTrainingReturnStartPose();
    bool trestTrainingGenerateTrajProximal();
    bool trestTrainingGenerateTrajDistal();
    bool trestTrainingUpdateEndPoseProximal();
    bool trestTrainingUpdateEndPoseDistal();
    bool trestTrainingStartTrajRecurr();
    bool trestTrainingEndTrajRecurr();

    bool trestTrainingIsReachedHomePose();
    bool trestTrainingIsReturnHomePose();
    bool trestTrainingIsGenerateTrajProximalComplete();
    bool trestTrainingIsGenerateTrajDistalComplete();

private:
	enum {
		HEADER_LEN = 6, CMD_TYPE_LEN = 4
	};

    TcpClient* tcp_client_;
    bool connected_;
    fd_set client_fd_set_;

    int recv_bytes_;
    uint8_t recv_buf_[TCP_COMM_BUF_SIZE];

    int send_bytes_;
    uint8_t send_buf_[TCP_COMM_BUF_SIZE];

    bool processCmd(ForceMasterProto::MessageType cmd_type);
    bool processSetCmd(ForceMasterProto::MessageType cmd_type, int level);
    bool processGetCmd(ForceMasterProto::MessageType cmd_type, int &level);
    bool processGetStatusCmd(ForceMasterProto::MessageType cmd_type);

    bool processCmdSend(ForceMasterProto::MessageType cmd_type);
    bool processCmdSendForSet(ForceMasterProto::MessageType cmd_type, int level);
    bool processCmdRecvForGetLevel(ForceMasterProto::MessageType cmd_type);
    bool processCmdRecv(ForceMasterProto::MessageType cmd_type);

    bool processCmdSendForSetCartPosition(ForceMasterProto::MessageType cmd_type);
    bool processRecvGetSensorStatus(ForceMasterProto::MessageType cmd_type);
    bool processRecvGetJointPosition(ForceMasterProto::MessageType cmd_type);
    bool processRecvGetCartPosition(ForceMasterProto::MessageType cmd_type);

    long long getCurrentTime();
};

#endif

