#include <cstring>
#include <unistd.h>

#include "sleep_milli.h"
#include "dashboard_comm.h"

#define DASHBOARD_COMM_BUF_SIZE 200

DashboardComm::DashboardComm(std::string host)
{
    connected_ = false;

    connect_times_ = 0;
    recv_bytes_ = 0;
    memset(buf_, 0, sizeof(buf_));

    host_ = host;
    tcp_client_ = new TcpClient(host_, DASHBOARD_PORT);
}

DashboardComm::~DashboardComm()
{
    if(tcp_client_ != nullptr)
    {
        delete tcp_client_;
        tcp_client_ = nullptr;
    }
}

bool DashboardComm::isConneted()
{
    return is_running_;
}

bool DashboardComm::start()
{
    if(!tcp_client_->inetSocket())
    {
        printf("DashboardComm: Tcp communication initial failed.\n");
        return false;
    }
    printf("DashboardComm: Tcp communication initial successful.\n");
    if(!tcp_client_->inetConnect())
    {
        tcp_client_->inetClose();
        printf("DashboardComm: Tcp communication connection failed.\n");
        return false;
    }
    printf("DashboardComm: Tcp communication connection successful.\n");
    if(!recvMessageOnce())
    {
        tcp_client_->inetClose();
        printf("DashboardComm: Tcp communication recv failed.\n");
        return false;
    }

    if(!ThreadBase<DashboardComm>::start(SCHED_RR))
    {
        printf("DashboardComm: create thread failed\n");
        return false;
    }

    printf("DashboardComm: create thread successful\n");

    is_running_ = true;
    connected_ = true; 
    return true;
}

void DashboardComm::halt()
{
    is_running_ = false;
    ThreadBase<DashboardComm>::stop();

    tcp_client_->inetClose();
    connected_ = false;
}

void DashboardComm::run()
{
	while (is_running_)
	{
        if(!tcp_client_->isConnected())
        {
            // robot_state_->setDisconnected(); to do...
            connected_ = false;
            printf("DashboardComm: tcp communication is disconected.\n");

            for (; connect_times_ < DASHBOARD_TCP_CONNECT_MAX_COUNT; ++connect_times_)
            {
                sleep(DASHBOARD_TCP_CONNECT_DEURING_SEC);
                printf("DashboardComm: Trying to connect: the %dth time.\n", connect_times_);
                if(!tcp_client_->inetConnect())
                {
                    printf("DashboardComm: Connection failed for the %dth time\n", connect_times_);
            
                    if(connect_times_ >= DASHBOARD_TCP_CONNECT_MAX_COUNT)
                    {
                        tcp_client_->inetClose();
                        is_running_ = false;
                        printf("DashboardComm: Connection failed, program exited\n");
                        return;
                    }

                    continue;
                }

                connected_ = true;
                printf("DashboardComm: Tcp communication connection success\n"); 
                break;
            }
        }
        //printf("DashboardComm: connected...\n");
        sleep_milli(1000);   
    }
}


bool DashboardComm::recvMessage(std::string& recv_msg)
{
    if(!connected_)
    {
        printf("DashboardComm: Tcp communication is disconected\n"); 
        return false;
    }

    if(!tcp_client_->inetRecv(recv_msg))
    {
        printf("DashboardComm: Failed to receive msg form dashboard.\n"); 
        return false;
    }
    return true;
}

bool DashboardComm::recvMessageOnce()
{
    string recv_msg;
    if(!tcp_client_->inetRecv(recv_msg))
    {
        printf("DashboardComm: Failed to receive msg form dashboard.\n"); 
        return false;
    }
    printf("DashboardComm: %s", recv_msg.c_str());
    return true;
}

bool DashboardComm::sendMessage(std::string send_msg)
{
    if(!connected_)
    {
        printf("DashboardComm: Tcp communication is disconected\n"); 
        return false;
    }

    if (send_msg.back() != '\n') 
    {
        send_msg.append("\n");
    }

    if(!tcp_client_->inetSend(send_msg))
    {
        printf("DashboardComm: Failed to send msg to dashboard.\n"); 
        return false;
    }

    return true;
}

bool DashboardComm::processCommand(std::string send_msg, std::string& recv_msg)
{
    printf("send_msg: %s\n", send_msg.c_str());
    if(!sendMessage(send_msg))
    {
        printf("DashboardComm: Failed to send message.\n"); 
        return false;
    }

    if(!recvMessage(recv_msg))
    {
        printf("DashboardComm: Failed to recv message.\n"); 
        return false;  
    }
    printf("recv_msg: %s\n", recv_msg.c_str());
    return true;
}

bool DashboardComm::processCommand(std::string send_msg, std::string recv_msg_expect, std::string& recv_msg)
{
    if(!processCommand(send_msg, recv_msg))
    {
        printf("DashboardComm: Failed to process command.\n"); 
        return false;
    }

    if(recv_msg.find(recv_msg_expect) == std::string::npos)
    {
        printf("DashboardComm: The received message does not match the expected message.\n"); 
        return false;
    }
    return true;
}
