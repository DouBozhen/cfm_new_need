#ifndef DASH_BOARD_COMM
#define DASH_BOARD_COMM

#include <string>
#include "thread_base.h"
#include "tcp_client.h"

#define DASHBOARD_PORT 29999
#define DASHBOARD_TCP_CONNECT_MAX_COUNT 10
#define DASHBOARD_TCP_CONNECT_DEURING_SEC 5
#define DASHBOARD_TCP_COMM_BUF_SIZE 200

class DashboardComm : public ThreadBase<DashboardComm>
{
public:
    DashboardComm(std::string host);
    ~DashboardComm();

    bool start();
    void halt();
    void run();
    bool isConneted();

    bool processCommand(std::string send_msg, std::string& recv_msg);
    bool processCommand(std::string send_msg, std::string recv_msg_expect, std::string& recv_msg);

private:
    bool connected_;
    int connect_times_;

    uint8_t buf_[DASHBOARD_TCP_COMM_BUF_SIZE];
    int recv_bytes_;

    std::string host_;
    TcpClient* tcp_client_;

    bool recvMessage(std::string& recv_msg);
    bool sendMessage(std::string send_msg);
    bool recvMessageOnce();
};

#endif