#include <sys/time.h>
#include <unistd.h>
#include <cstring>
#include <fstream>

#include "thread_pool.h"
#include "sleep_milli.h"
#include "path_file.h"
#include "arm_comm_rt.h"


// std::ofstream ofile("../../../docs/time_out.txt");
std::ofstream ofile(TIMEOUT_FILE_DIR, fstream::out|ios_base::trunc);
ArmCommRealtime::ArmCommRealtime(std::string host)
{
	arm_state_rt_ = new ArmStateRealtime();

    connect_times_ = 0;
    recv_bytes_ = 0;
    memset(buf_, 0, sizeof(buf_));

    host_ = host;
    tcp_client_ = new TcpClient(host_, RT_PORT);

	connected_ = false;
	//is_running_ = false;

	safety_count_ = SAFETY_COUNT_MAX + 1;
}

ArmCommRealtime::~ArmCommRealtime()
{
    if(tcp_client_ != nullptr)
    {
        delete tcp_client_;
        tcp_client_ = nullptr;
    }

    if(arm_state_rt_ != nullptr)
    {
        delete arm_state_rt_;
        arm_state_rt_ = nullptr;
    }  
}

bool ArmCommRealtime::start()
{
    if(!tcp_client_->inetSocket())
    {
        printf("ArmCommRealtime: Tcp communication initial failed.\n");
        return false;
    }

    if(!tcp_client_->inetConnect())
    {
        tcp_client_->inetClose();
        printf("ArmCommRealtime: Tcp communication connection failed.\n");
        return false;
    }

    if(!ThreadBase<ArmCommRealtime>::start(SCHED_RR))
    {
        printf("ArmCommRealtime: create thread failed\n");
        return false;
    }

    printf("ArmCommRealtime: create thread successful\n");

    is_running_ = true;
    connected_ = true;
    return true;
}

void ArmCommRealtime::halt() 
{
    is_running_ = false;

    tcp_client_->inetClose();
    connected_ = false;

    ThreadBase<ArmCommRealtime>::stop();
    ofile.close();
}

long long ArmCommRealtime::getCurrentTime()
{
    struct timeval time;
 
    gettimeofday(&time, NULL);
    
    return (time.tv_sec * 1000000 + time.tv_usec);
}

void ArmCommRealtime::rtNotifyCall()
{
    ThreadPool::getInstance().notifyCall();
}

void ArmCommRealtime::run()
{
	int bytes_read = 0;
	int offset = 0;

    long long start_time = 0;
    long long end_time = 0;
    int32_t LEN = 0;
    start_time = getCurrentTime();
    while(is_running_)
    {
        connected_ = tcp_client_->isConnected();

        LEN = TCP_COMM_BUF_SIZE - offset;
        if(connected_)
        {
            recv_bytes_ = 0;
            recv_bytes_ = tcp_client_->inetRecv(buf_, LEN);
            // printf("recv_bytes_ = %d\n", recv_bytes_);
            if(recv_bytes_ <= 0)
            {
                // tcp_client_->inetClose();
                connected_ = false;
                offset = 0;
                LEN = 0;
                bytes_read = 0;

                printf("ArmCommRealtime: tcp communication is disconected.\n");
                continue;
            }

            /* to do... */
			offset += recv_bytes_;
			uint8_t* ptr = buf_;
			while (offset > TCP_COMM_PACKAGE_LEN)
			{
				offset -= TCP_COMM_PACKAGE_LEN;
				ptr += TCP_COMM_PACKAGE_LEN;
			}
			if (0 < offset && offset < TCP_COMM_PACKAGE_LEN)
			{
				//memmove(buf_, ptr, offset);
                char recv_temp_char[TCP_COMM_PACKAGE_LEN];
                memcpy(recv_temp_char, (char*)buf_, recv_bytes_);
                recv_temp_char[recv_bytes_] = '\n';
                string recv_temp_string = recv_temp_char;
				printf("ArmCommRealtime: offset = %d, The connection is broken: %s\n", offset, recv_temp_char);//断开///后期可能加重连机制

				connected_ = false;
                offset = 0;
                LEN = 0;
                bytes_read = 0;
                continue;
			}

			if (offset == TCP_COMM_PACKAGE_LEN)
			{
				arm_state_rt_->decode(ptr);
				rtNotifyCall();

				if (safety_count_ == SAFETY_COUNT_MAX) 
                {
                    ur_data_type::Joint joint_vel;
                    memset(&joint_vel, 0, sizeof(joint_vel));
                    speedJointMove(joint_vel);
				}
				safety_count_ += 1;

		        end_time = getCurrentTime();
                int during = static_cast<int>((end_time - start_time) / 1000);

                start_time = end_time;
                ofile << during << std::endl;
				if (during < 5)
				{
                    //sleep_milli(5 - during);
				}
                else if(during > 8)
                {
					ofile << during << std::endl;
                }
				offset = 0;
			}
        }
        else
        {
            sleep(1);
            connect_times_++;
            printf("ArmCommRealtime: Trying to connect: the %dth time.\n", connect_times_);

            if(!tcp_client_->inetConnect())
            {
                printf("ArmCommRealtime: Connection failed for the %dth time\n", connect_times_);
           
                if(connect_times_ > TCP_CONNECT_MAX_COUNT)
                {
                    /* useless */
                    ur_data_type::Joint joint_vel;
                    memset(&joint_vel, 0, sizeof(joint_vel));
                    speedJointMove(joint_vel);

                    halt();
                    printf("ArmCommRealtime: Connection failed, program exited\n");
                    return;
                }

                continue;
            }

            connected_ = true;
            printf("ArmCommRealtime: Tcp communication connection success\n"); 
        }
    }
}

bool ArmCommRealtime::recvMessage(std::string& recv_msg)
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

bool ArmCommRealtime::sendMessage(std::string send_msg)
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
    // printf("ArmCommRealtime: send \n%s\n", send_msg.c_str());
    return true;
}

bool ArmCommRealtime::isConnected()
{
    return is_running_;
}
