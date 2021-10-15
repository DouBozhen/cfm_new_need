#ifndef ARM_COMM_H
#define ARM_COMM_H

#include <pthread.h>
#include <string>

#include "thread_base.h"
#include "tcp_client.h"
#include "arm_state.h"

#define PRI_PORT 30001
#define ARM_COMM_BUF_SIZE 11080
#define ARM_COMM_OVERALL_PACKAGE_LEN 4
#define ARM_TCP_CONNECT_MAX_COUNT 10

class ArmComm : public ThreadBase<ArmComm>
{
public:
    ArmComm(std::string host);
    ~ArmComm();

    bool start();
    void halt();

    void run();
    bool isRunning();

    ArmState* arm_state_;
    bool connected_;

private:
    // bool is_running_;

    int connect_times_;

    uint8_t buf_[ARM_COMM_BUF_SIZE];
    int recv_bytes_;

    std::string host_;
    TcpClient* tcp_client_;

    uint32_t littileToBig32(uint32_t data)
    {
        return
            (((uint32_t)(data) & 0x000000ff) << 24) |
            (((uint32_t)(data) & 0x0000ff00) << 8) |
            (((uint32_t)(data) & 0x00ff0000) >> 8) |
            (((uint32_t)(data) & 0xff000000) >> 24);
    }

    uint32_t bigToLittile32(uint32_t data)
    {
        return 
            (((uint32_t)(data) & 0xff000000) >> 24) |
            (((uint32_t)(data) & 0x00ff0000) >> 8) |
            (((uint32_t)(data) & 0x0000ff00) << 8) |
            (((uint32_t)(data) & 0x000000ff) << 24);
    }
};

#endif