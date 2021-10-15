#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>

#include "thread_base.h"
#include "sleep_milli.h"

#include "arm_comm.h"

using namespace std;

ArmComm::ArmComm(string host)
{
    connected_ = false;
    //is_running_= false; // to do...

    connect_times_ = 0;
    recv_bytes_ = 0;
    memset(buf_, 0, sizeof(buf_));

    host_ = host;
    tcp_client_ = new TcpClient(host_, PRI_PORT);
    arm_state_ = new ArmState();
}

ArmComm::~ArmComm()
{
    if(tcp_client_ != nullptr)
    {
        delete tcp_client_;
        tcp_client_ = nullptr;
    }

    if(arm_state_ != nullptr)
    {
        delete arm_state_;
        arm_state_ = nullptr;
    }
}

bool ArmComm::start()
{
    if(!tcp_client_->inetSocket())
    {
        printf("ArmComm: Tcp communication initial failed.\n");
        return false;
    }

    if(!tcp_client_->inetConnect())
    {
        tcp_client_->inetClose();
        printf("ArmComm: Tcp communication connection failed.\n");
        return false;
    }

    if(!ThreadBase<ArmComm>::start(SCHED_RR))
    {
        printf("ArmComm: create thread failed\n");
        return false;
    }

    printf("ArmComm: create thread successful\n");
#if 0
    if(tcp_client_->inetRecv(buf_, ARM_COMM_OVERALL_PACKAGE_LEN) < 0)
    {
        printf("ArmComm: recv data failed\n");
        return false;
    }
#endif

    is_running_ = true;
    connected_ = true;
    return true;
}

void ArmComm::halt()
{
    is_running_ = false;

    tcp_client_->inetClose();
    connected_ = false;

    ThreadBase<ArmComm>::stop();
}

bool ArmComm::isRunning()
{
    return is_running_;
}

void ArmComm::run()
{
    int offset = 0;
    int LEN = 0;
    
	while (is_running_)
	{
        connected_ = tcp_client_->isConnected();
        if(connected_)
        {
            // LEN = ARM_COMM_BUF_SIZE - offset; // to do ...
            memset(buf_, 0, ARM_COMM_BUF_SIZE);
            recv_bytes_ = tcp_client_->inetRecv(buf_, ARM_COMM_OVERALL_PACKAGE_LEN);
            if(recv_bytes_ == ARM_COMM_OVERALL_PACKAGE_LEN)
            {
                //int pkg_len = 0;
                //memcpy(&pkg_len, buf_, ARM_COMM_OVERALL_PACKAGE_LEN);
                int pkg_len = *((int*)buf_);
                pkg_len = ntohl(pkg_len);

                recv_bytes_ = tcp_client_->inetRecv(buf_ + ARM_COMM_OVERALL_PACKAGE_LEN, pkg_len - ARM_COMM_OVERALL_PACKAGE_LEN);
                arm_state_->decode(buf_ + ARM_COMM_OVERALL_PACKAGE_LEN, pkg_len - ARM_COMM_OVERALL_PACKAGE_LEN); 
            }
            else
            {
                arm_state_->setDisconnected(); 
    
                tcp_client_->inetClose();
                connected_ = false;
                printf("ArmComm: tcp communication is disconected.\n");
                continue;
            }

            sleep_milli(100);
        }
        else
        {
            sleep(10);
            connect_times_++;
            printf("ArmComm: Trying to connect: the %dth time.\n", connect_times_);
            if(!tcp_client_->inetConnect())
            {
                printf("ArmComm: Connection failed for the %dth time\n", connect_times_);
           
                if(connect_times_ > ARM_TCP_CONNECT_MAX_COUNT)
                {
                    tcp_client_->inetClose();
                    printf("ArmComm: Connection failed, program exited\n");
                    return;
                }

                continue;
            }
            
            connected_ = true;
            printf("ArmComm: Tcp communication connection success\n"); 
        }
    }
}

