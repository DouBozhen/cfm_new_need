#include <unistd.h>
#include <sys/time.h>
#include "test_tcp_comm.h"

using namespace ForceMasterProto;

bool TestTcpComm::testHeartBeat()
{
    return processCmd(HeartBeat);
}

bool TestTcpComm::testPowerOff()
{
    return processCmd(Poweroff);
}

bool TestTcpComm::testStopRobot()
{
    return processCmd(CloseRobot);
}

bool TestTcpComm::testSetForceLevel(int level)
{
    return processSetCmd(TractionForce_SetLevel, level);
}

bool TestTcpComm::testGetForceLevel()
{
    int speed_ratio;
    return processGetCmd(TractionForce_GetLevel, speed_ratio);
}

bool TestTcpComm::testSetRepeatedCount(int count)
{
    return processSetCmd(TrajectoryReconstruction_SetRepeatCount, count);
}

bool TestTcpComm::testGetRepeatedCount()
{
    int speed_ratio;
    return processGetCmd(TrajectoryReconstruction_GetRepeatCount, speed_ratio);
}

bool TestTcpComm::testSetSpeedRatio(int ratio)
{
    return processSetCmd(TrajectoryReconstruction_SetSpeedRatio, ratio);
}

bool TestTcpComm::testGetSpeedRatio()
{
    int speed_ratio;
    return processGetCmd(TrajectoryReconstruction_GetSpeedRatio, speed_ratio);
}

bool TestTcpComm::testOpenTrajRecurr()
{
    return processCmd(TrajectoryReconstruction_Open);
}

bool TestTcpComm::testCloseTrajRecurr()
{
    return processCmd(TrajectoryReconstruction_Close);
}

bool TestTcpComm::testStartTrajRecurr()
{
    return processCmd(TrajectoryReconstruction_Start);
}

bool TestTcpComm::testStopTrajRecurr()
{
    return processCmd(TrajectoryReconstruction_End);
}

bool TestTcpComm::isConnected()
{
    return tcp_client_->isConnected();
}

bool TestTcpComm::processCmd(ForceMasterProto::MessageType cmd_type)
{
    uint32_t cmd_type_32 = (uint32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, sizeof(cmd_type_32));

    uint16_t data_len = 0;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, sizeof(data_len));

    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, HEADER_LEN))
    {
        printf("tcp client send data.\n");
        return false;
    }
    // usleep(1000000); /* 1s */

    int recv_len = 0;
    while (recv_len <= 0)
    {
        memset(&recv_buf_, 0, sizeof(recv_buf_));
        recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);

        printf("recv_len = %d\n", recv_len);
        usleep(100000);
    }    

    if (recv_len < -1)
    {
        printf("tcp client recv data.\n");
        return false;
    }

    memcpy(&cmd_type_32, recv_buf_, CMD_TYPE_LEN);
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error. %d : \n", type, cmd_type);
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);

    if (data_len > 0)
    {
        recv_len = 0; // = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        while (recv_len <= 0)
        {
            recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
            usleep(100000);
        }    
     
        if (recv_len < -1)
        {
            printf("tcp client send data.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseCommon response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error. data_len = %d\n", data_len);
        return false;
    }

    if (!response.is_successful())
    {
        printf("recv data: %s\n", response.error_str().c_str());
    }
    else 
    {
        printf("recv data: %s\n", response.error_str().c_str());
    }

    return true;
}

bool TestTcpComm::processGetStatusCmd(ForceMasterProto::MessageType cmd_type)
{
    uint32_t cmd_type_32 = (uint32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, sizeof(cmd_type_32));

    uint16_t data_len = 0;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, sizeof(data_len));

    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, HEADER_LEN))
    {
        printf("tcp client send data.\n");
        return false;
    }
    // usleep(1000000); /* 1s */

    int recv_len = 0;
    while (recv_len == 0)
    {
        recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);
        usleep(100000);
    }    

    if (recv_len < -1)
    {
        printf("tcp client send data.\n");
        return false;
    }

    memcpy(&cmd_type_32, recv_buf_, sizeof(cmd_type_32));
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error.\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], sizeof(data_len));

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client send data.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseStatus response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.header().is_successful())
    {
        printf("recv data: %s\n", response.header().error_str().c_str());
    }
    else 
    {
        printf("recv data: status = %d\n", response.status());
    }

    return true; 
}

bool TestTcpComm::processGetCmd(ForceMasterProto::MessageType cmd_type, int &level)
{
    uint32_t cmd_type_32 = (uint32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, sizeof(cmd_type_32));

    uint16_t data_len = 0;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, sizeof(data_len));

    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, HEADER_LEN))
    {
        printf("tcp client send data.\n");
        return false;
    }
    // usleep(1000000); /* 1s */

    int recv_len = 0;
    while (recv_len == 0)
    {
        recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);
        usleep(100000);
    }    

    if (recv_len < -1)
    {
        printf("tcp client send data.\n");
        return false;
    }

    memcpy(&cmd_type_32, recv_buf_, sizeof(cmd_type_32));
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error.\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], sizeof(data_len));

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client send data.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseLevel response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.header().is_successful())
    {
        printf("recv data: %s\n", response.header().error_str().c_str());
    }
    else 
    {
        printf("recv data: level = %d\n", response.level());
    }

    return true;
}

bool TestTcpComm::processSetCmd(ForceMasterProto::MessageType cmd_type, int level)
{
    Level send_level;
    send_level.set_level(level);
    int proto_data_len = send_level.ByteSize();
    send_level.SerializeToArray(&send_buf_[HEADER_LEN], proto_data_len);
    printf("proto_data_len = %d\n", proto_data_len);
    uint32_t cmd_type_32 = (int32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, sizeof(cmd_type_32));

    uint16_t data_len = proto_data_len;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, HEADER_LEN - CMD_TYPE_LEN);

    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, HEADER_LEN))
    {
        printf("tcp client send data.\n");
        return false;
    }
    // usleep(1000000); /* 1s */

    int recv_len = 0;
    while (recv_len == 0)
    {
        recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);
        // usleep(100000);
    }    

    if (recv_len < -1)
    {
        printf("tcp client send data.\n");
        return false;
    }

    memcpy(&cmd_type_32, recv_buf_, sizeof(cmd_type_32));
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error.\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], sizeof(data_len));

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client send data.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseCommon response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.is_successful())
    {
        printf("recv data: %s\n", response.error_str().c_str());
    }
    else 
    {
        printf("recv data: %s\n", response.error_str().c_str());
    }

    return true;
}

bool TestTcpComm::processCmdSend(ForceMasterProto::MessageType cmd_type)
{
    uint32_t cmd_type_32 = (uint32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, sizeof(cmd_type_32));

    uint16_t data_len = 0;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, sizeof(data_len));

    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, HEADER_LEN))
    {
        printf("tcp client send data.\n");
        return false;
    }
    printf("send successful\n");
    return true;
}

bool TestTcpComm::processCmdSendForSet(ForceMasterProto::MessageType cmd_type, int level)
{
    Level send_level;
    send_level.set_level(level);
    int proto_data_len = send_level.ByteSize();
    send_level.SerializeToArray(&send_buf_[HEADER_LEN], proto_data_len);
    printf("proto_data_len = %d\n", proto_data_len);

    uint32_t cmd_type_32 = (int32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, CMD_TYPE_LEN);

    uint16_t data_len = proto_data_len;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, HEADER_LEN - CMD_TYPE_LEN);
    
    int send_buf_size = HEADER_LEN + data_len;
    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, send_buf_size))
    {
        printf("tcp client send data.\n");
        return false;
    }
    return true;
}

bool TestTcpComm::processCmdSendForSetCartPosition(ForceMasterProto::MessageType cmd_type)
{
    CartPosition pose;
    pose.set_x(0.1);
    pose.set_y(0.1);
    pose.set_z(0.1);
    pose.set_a(0.1);
    pose.set_b(0.1);
    pose.set_c(0.1);

    int proto_data_len = pose.ByteSize();
    pose.SerializeToArray(&send_buf_[HEADER_LEN], proto_data_len);
    printf("proto_data_len = %d\n", proto_data_len);

    uint32_t cmd_type_32 = (int32_t)cmd_type;
    memcpy(send_buf_, &cmd_type_32, CMD_TYPE_LEN);

    uint16_t data_len = proto_data_len;
    memcpy(&send_buf_[CMD_TYPE_LEN], &data_len, HEADER_LEN - CMD_TYPE_LEN);
    
    int send_buf_size = HEADER_LEN + data_len;
    if (!isConnected())
    {
        printf("tcp client disconnect.\n");
        return false;
    }
    if (!tcp_client_->inetSend(send_buf_, send_buf_size))
    {
        printf("tcp client send data.\n");
        return false;
    }
    printf("-------- send ok ------------\n");
    return true;
}

bool TestTcpComm::processCmdRecvForGetLevel(ForceMasterProto::MessageType cmd_type)
{
    int recv_len = 0;
    recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);

    if (recv_len < -1)
    {
        printf("tcp client recv data error.\n");
        return false;
    }

    if (recv_len == 0)
    {
        printf("服务器推出.\n");
        return false;
    }

    uint32_t cmd_type_32 = 0;
    uint16_t data_len = 0;
    memcpy(&cmd_type_32, recv_buf_, CMD_TYPE_LEN);
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error. %d : %d\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client recv data error.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseLevel response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.header().is_successful())
    {
        printf("recv data: %s\n", response.header().error_str().c_str());
    }
    else 
    {
        printf("recv data: level = %d\n", response.level());
    }

    return true;
}

bool TestTcpComm::processCmdRecv(ForceMasterProto::MessageType cmd_type)
{
    int recv_len = 0;
    memset(recv_buf_, 0, sizeof(recv_buf_));
    recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);

    if (recv_len < -1)
    {
        printf("tcp client recv data error.\n");
        return false;
    }

    if (recv_len == 0)
    {
        printf("服务器推出.\n");
        return false;
    }

    uint32_t cmd_type_32 = 0;
    uint16_t data_len = 0;

    memcpy(&cmd_type_32, recv_buf_, CMD_TYPE_LEN);
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error. %d : %d\n", type, cmd_type);
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client recv data error.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseCommon response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.is_successful())
    {
        printf("recv data: %s\n", response.error_str().c_str());
    }
    else 
    {
        printf("recv data: %s\n", response.error_str().c_str());
    }

    return true;
}

bool TestTcpComm::processRecvGetSensorStatus(ForceMasterProto::MessageType cmd_type)
{
    int recv_len = 0;
    recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);

    if (recv_len < -1)
    {
        printf("tcp client recv data error.\n");
        return false;
    }

    if (recv_len == 0)
    {
        printf("服务器推出.\n");
        return false;
    }

    uint32_t cmd_type_32 = 0;
    uint16_t data_len = 0;
    memcpy(&cmd_type_32, recv_buf_, CMD_TYPE_LEN);
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error. %d : %d\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client recv data error.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseSensorStatus response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.header().is_successful())
    {
        printf("recv data: %s\n", response.header().error_str().c_str());
    }
    else 
    {
        printf("recv data: status = %d\n", response.status());
    }

    return true;
}

bool TestTcpComm::processRecvGetJointPosition(ForceMasterProto::MessageType cmd_type)
{
    int recv_len = 0;
    recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);

    if (recv_len < -1)
    {
        printf("tcp client recv data error.\n");
        return false;
    }

    if (recv_len == 0)
    {
        printf("服务器推出.\n");
        return false;
    }

    uint32_t cmd_type_32 = 0;
    uint16_t data_len = 0;
    memcpy(&cmd_type_32, recv_buf_, CMD_TYPE_LEN);
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error. %d : %d\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client recv data error.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseJointPosition response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.header().is_successful())
    {
        printf("recv data: %s\n", response.header().error_str().c_str());
    }
    else 
    {
        printf("recv data: joint_num = %d\n", response.position().joint_num());
        for (int i = 0; i != response.position().joint_num(); ++i)
        {
            printf("recv data: joint[%d] = %f\n", i, response.position().joint(i));
        }
    }

    return true;
}

bool TestTcpComm::processRecvGetCartPosition(ForceMasterProto::MessageType cmd_type)
{
    int recv_len = 0;
    recv_len = tcp_client_->inetRecv(recv_buf_, HEADER_LEN);

    if (recv_len < -1)
    {
        printf("tcp client recv data error.\n");
        return false;
    }

    if (recv_len == 0)
    {
        printf("服务器推出.\n");
        return false;
    }

    uint32_t cmd_type_32 = 0;
    uint16_t data_len = 0;
    memcpy(&cmd_type_32, recv_buf_, CMD_TYPE_LEN);
    MessageType type = (MessageType)cmd_type_32;
    if (type != cmd_type)
    {
        printf("tcp client recv data: cmd error. %d : %d\n");
        return false;
    }

    memcpy(&data_len, &recv_buf_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);

    if (data_len > 0)
    {
        recv_len = tcp_client_->inetRecv(&recv_buf_[HEADER_LEN], data_len);
        if (recv_len < -1)
        {
            printf("tcp client recv data error.\n");
            return false;
        }
        else
        {
            printf("recv_len %d： data_len %d\n", recv_len , data_len);
        }
    }

    ResponseCartPosition response;
	if (!response.ParseFromArray(&recv_buf_[HEADER_LEN], data_len))
	{
        printf("tcp client recv data: parse response error.\n");
        return false;
    }

    if (!response.header().is_successful())
    {
        printf("recv data: %s\n", response.header().error_str().c_str());
    }
    else 
    {
        printf("recv data: %lf, %lf, %lf, %lf, %lf, %lf\n", 
            response.position().x(),
            response.position().y(),
            response.position().z(),
            response.position().a(),
            response.position().b(),
            response.position().c()
        );
    }

    return true;
}
