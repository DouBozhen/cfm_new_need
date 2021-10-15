
#include <unistd.h>
#include <sys/time.h>
#include "test_tcp_comm.h"

using namespace ForceMasterProto;


TestTcpComm::TestTcpComm()
{
    recv_bytes_ = 0;
    memset(recv_buf_, 0, sizeof(recv_buf_));

    send_bytes_ = 0;
    memset(send_buf_, 0, sizeof(send_buf_));

    tcp_client_ = new TcpClient("127.0.0.1", CLIENT_PORT, false);
    connected_ = false;
}

TestTcpComm::~TestTcpComm()
{
    if(tcp_client_ != nullptr)
    {
        tcp_client_->inetClose();

        delete tcp_client_;
        tcp_client_ = nullptr;
    }
}

bool TestTcpComm::init()
{
    if(!tcp_client_->inetSocket())
    {
        printf("TestTcpComm: Tcp communication initial failed.\n");
        return false;
    }
    else 
    {
        printf("TestTcpComm: Tcp communication initial successful.\n");  
    }

    if(!tcp_client_->inetConnect())
    {
        tcp_client_->inetClose();
        printf("TestTcpComm: Tcp communication connection failed.\n");
        return false;
    }
    else 
    {
        printf("TestTcpComm: Tcp communication connection successful.\n");  
    }

    connected_ = true;

    int server_sock_fd = 0;
    struct timeval timeout;
    timeout.tv_sec = 0;
    timeout.tv_usec = 10000;

    tcp_client_->getSocket(server_sock_fd);
    if (setsockopt(server_sock_fd, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval)) != 0)
    {
       printf("set accept timeout failed");
    }

#if 1
    FD_ZERO(&client_fd_set_);
    FD_SET(STDIN_FILENO, &client_fd_set_);
    int socket = 0;
    tcp_client_->getSocket(socket);
    FD_SET(socket, &client_fd_set_);
#endif
    return true;
}

bool TestTcpComm::testAllParams()
{
    int count = 10000;
    int i = 0;
    long long start_time = 0;
    int max_during = 0;
    while (i != count)
    {
        printf("-------------- start --------------\n");

        i++;
        struct timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 10000;
        fd_set client_fd_set;
        FD_ZERO(&client_fd_set);
        //FD_SET(STDIN_FILENO, &client_fd_set);

        int server_sock_fd = 0;
        tcp_client_->getSocket(server_sock_fd);
        FD_SET(server_sock_fd, &client_fd_set);
        select(server_sock_fd + 1, &client_fd_set, NULL, NULL, &tv);
        printf("FD_ISSET \n");
        start_time = getCurrentTime();
        processCmdSendForSetCartPosition(SetCartPosition);
  
        //usleep(1000000);
        tcp_client_->getSocket(server_sock_fd);
        FD_SET(server_sock_fd, &client_fd_set);
        if(FD_ISSET(server_sock_fd, &client_fd_set))
        {
            processCmdRecv(SetCartPosition);
        }
        long long end_time = getCurrentTime();
        int during = static_cast<int>((end_time - start_time) / 1000);
        printf("-------------- %d: %d ms--------------\n", i, during);
       
        if (max_during < during)
        {
            max_during = during;
        }
        // usleep(10000);
    }
    printf("--------------end: max during %d--------------\n", max_during);
       
    return true;
}

long long TestTcpComm::getCurrentTime()
{
    struct timeval time;
 
    gettimeofday(&time, NULL);
    
    return (time.tv_sec * 1000000 + time.tv_usec);
}
