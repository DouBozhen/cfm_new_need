#include <errno.h>  
#include <fcntl.h> 
#include <unistd.h>
#include <cstring>
#include <ctime>
#include "rpc_connection.h"

using namespace ForceMasterProto;
using namespace std;

RpcConnection::RpcConnection(int socket, fd_set &server_fd_set)
{
    mutex_ = PTHREAD_MUTEX_INITIALIZER;
    rpc_table_mutex_ = PTHREAD_MUTEX_INITIALIZER;

    s_ = socket;
	server_fd_set_ptr_ = &server_fd_set;

	is_conning_ = false;
	is_exit_ = false;

	is_concordance_ = true;
	is_received_ = false;

	recv_buffer_size_ = DATA_BUFFER_LEN;
	send_buffer_size_ = DATA_BUFFER_LEN;
	recv_buffer_ = new char[recv_buffer_size_]();
	send_buffer_ = new char[send_buffer_size_]();

	memset(recv_buffer_, 0, recv_buffer_size_);
	memset(send_buffer_, 0, send_buffer_size_);
    recv_times_ = 0;
    send_times_ = 0;

	rpc_table_.clear();
	initRpcTable();
}

RpcConnection::~RpcConnection()
{
    is_conning_ = false;

    void* res;
    pthread_join(recv_thread_, &res);
    pthread_join(send_thread_, &res);
    pthread_mutex_destroy(&mutex_);
    pthread_mutex_destroy(&rpc_table_mutex_);
	rpc_table_.clear();

    if (recv_buffer_ != nullptr)
	{
		delete[] recv_buffer_;
		recv_buffer_ = nullptr;
	}
	if (send_buffer_ != nullptr)
	{
		delete[] send_buffer_;
		send_buffer_ = nullptr;
	}
}

int RpcConnection::getConnSocket()
{
	return s_;
}

bool RpcConnection::startRunning()
{
	is_conning_ = true;

	int ret = pthread_create(&recv_thread_, nullptr, recvMessageThread, (void*)this);
    if(ret < 0)
    {
        printf("RpcConnection: create recv thread failed, %d\n", ret);
        return false;
    }

	ret = pthread_create(&send_thread_, nullptr, sendMessageThread, (void*)this);
    if(ret < 0)
    {
        printf("RpcConnection: create send thread failed, %d\n", ret);
        return false;
    }

	cout << "Socket is connecting to client." << endl;
	return true;
}

void RpcConnection::disConning()
{
    is_conning_ = false;
}

bool RpcConnection::isConning()
{
    return is_conning_;
}

bool RpcConnection::isExit()
{
	return is_exit_;
}

void* RpcConnection::recvMessageThread(void* param)
{
	RpcConnection* conn = static_cast<RpcConnection*>(param);
	int recv_bytes = 0;

	// struct timeval timeout = {1,0};
	// if (setsockopt(conn->s_, SOL_SOCKET, SO_RCVTIMEO, (char *)&timeout, sizeof(struct timeval)) != 0)
	// {
		// printf("RpcConnection: set recv timeout failed");
	// }

	while (conn->is_conning_)
	{
		if (!FD_ISSET(conn->s_, conn->server_fd_set_ptr_))
		{
			usleep(THREAD_TIMEOUT);
			continue;
		}
#if 0
		if (conn->is_received_)
		{
			usleep(THREAD_TIMEOUT);
			continue;
		}
#endif
		memset(conn->recv_buffer_, 0, DATA_BUFFER_LEN);

		// recv header
		recv_bytes = recv(conn->s_, conn->recv_buffer_, HEADER_LEN, 0);
		// if (recv_bytes == 0 ) continue;

		if(recv_bytes < 0)
		{
            printf("RpcConnection: recv failed, %s(errno: %d)",strerror(errno), errno); 
            if (conn->recv_times_ < RECV_REPEATED_COUNT)
            {
                conn->recv_times_++;
                continue;
            }
 
            conn->recv_times_ = 0;	
            break;
		}

		if (recv_bytes == 0)
		{
			printf("The connection exited\n");
			conn->disConning();
			continue;
		}

		uint16_t data_len = 0;
		memcpy(&data_len, &conn->recv_buffer_[CMD_TYPE_LEN], HEADER_LEN - CMD_TYPE_LEN);
		if (!conn->is_concordance_)
		{
			data_len = conn->bigToLittile32(data_len);
		}
		if (0 < data_len)
		{
			recv_bytes = recv(conn->s_, &conn->recv_buffer_[HEADER_LEN], data_len, 0);
			for (int i = 0; i != recv_bytes; ++i)
			{
				// printf("recv_buffer_[%d]: 0x%x\n", i + HEADER_LEN, (int)conn->recv_buffer_[HEADER_LEN + i]);
			}

            if (recv_bytes != data_len || (recv_bytes < 0 && errno != 11))
            {
                printf("RpcConnection: recv failed, %s(errno: %d)",strerror(errno), errno); 
                continue;
            }
		}

		conn->recv_buffer_size_ = HEADER_LEN + data_len;

		if (HEADER_LEN <= conn->recv_buffer_size_) // process data
		{
			uint32_t cmd_type;
			memcpy(&cmd_type, conn->recv_buffer_, CMD_TYPE_LEN);

			if (!conn->is_concordance_)
			{
				cmd_type = conn->bigToLittile32(cmd_type);
			}
			MessageType type = (MessageType)cmd_type;
			// conn->getTime();

			HandleFuncPtr func_ptr = conn->getHandlerByCmdType(type);
			if (func_ptr == nullptr)
			{
				string err = "Command is bot find";
				conn->handleErrorFunc(err, type);
			}
			else
			{
				(void)(conn->*func_ptr)(data_len);
			}
		}
		else //for test
		{
			string err = "Request data error";
			conn->handleErrorFunc(err, ForceMasterProto::None);
		}

		conn->is_received_ = true;
		usleep(THREAD_TIMEOUT);
	}

	conn->is_conning_ = false; // notify send thread to exit
	conn->is_received_ = true;
    printf("RpcConnection: closed!\n");

	return nullptr;
}

void* RpcConnection::sendMessageThread(void* param)
{
    RpcConnection* conn = static_cast<RpcConnection*>(param);

	while (conn->is_conning_)
	{
        if (!conn->is_received_)
        {
            usleep(THREAD_TIMEOUT);
            continue;
        }

        if (!conn->is_conning_)
        {
            conn->is_exit_ = true;
            break;
        }
        int send_len = HEADER_LEN;
        pthread_mutex_lock(&conn->mutex_);

        int bytes = send(conn->s_, (char*)conn->send_buffer_, conn->send_buffer_size_, 0);
        if (bytes < 0)
        {
            pthread_mutex_unlock(&conn->mutex_);
            if (conn->send_times_ < SEND_REPEATED_COUNT)
            {
                conn->send_times_++;
                continue;
            }
            else 
            {
                conn->send_times_ = 0;
                break;
            }
        }

        memset(conn->send_buffer_, 0, conn->DATA_BUFFER_LEN);
        conn->send_buffer_size_ = 0;
        pthread_mutex_unlock(&conn->mutex_);

        conn->is_received_ = false;
		usleep(THREAD_TIMEOUT);
	}

	conn->is_received_ = true;
    conn->is_conning_ = false;
    conn->is_exit_ = true;

	return nullptr;
}

RpcConnection::HandleFuncPtr RpcConnection::getHandlerByCmdType(ForceMasterProto::MessageType cmd_type)
{
    pthread_mutex_lock(&rpc_table_mutex_);
	for (unsigned int i = 0; i < rpc_table_.size(); ++i)
	{
		if (rpc_table_[i].cmd_type == cmd_type)
		{
            pthread_mutex_unlock(&rpc_table_mutex_);
			return rpc_table_[i].request_func_ptr;
		}
	}
    pthread_mutex_unlock(&rpc_table_mutex_);
	return NULL;
}

uint16_t RpcConnection::bigToLittile16(uint16_t data)
{
	return 
		(((uint16_t)(data) & 0xff00) >> 8) | 
		(((uint16_t)(data) & 0x00ff) << 8);
}
uint32_t RpcConnection::bigToLittile32(uint32_t data)
{
	return 
		(((uint32_t)(data) & 0xff000000) >> 24) |
		(((uint32_t)(data) & 0x00ff0000) >> 8) |
		(((uint32_t)(data) & 0x0000ff00) << 8) |
		(((uint32_t)(data) & 0x000000ff) << 24);
}

uint16_t RpcConnection::littileToBig16(uint16_t data)
{
	return
	(((uint16_t)(data) & 0x00ff) << 8) |
		(((uint16_t)(data) & 0xff00) >> 8);
}

uint32_t RpcConnection::littileToBig32(uint32_t data)
{
	return
		(((uint32_t)(data) & 0x000000ff) << 24) |
		(((uint32_t)(data) & 0x0000ff00) << 8) |
		(((uint32_t)(data) & 0x00ff0000) >> 8) |
		(((uint32_t)(data) & 0xff000000) >> 24);
}

void RpcConnection::setSendBuffferSize(int size)
{
	send_buffer_size_ = size;
}

void RpcConnection::cmdTypeToBuffer(const uint32_t &cmd_type)
{
    memcpy(send_buffer_, &cmd_type, CMD_TYPE_LEN);
}

void RpcConnection::dataLenToBuffer(const uint16_t& len)
{
    memcpy(&send_buffer_[CMD_TYPE_LEN], &len, HEADER_LEN - CMD_TYPE_LEN);
}

void RpcConnection::headerToBuffer(ForceMasterProto::MessageType cmd_type, uint16_t data_len)
{
   /* process cmd */
	uint32_t type = convertResponseCmdType(cmd_type);
	cmdTypeToBuffer(type);

    /* process msg len */
	data_len = convertResponseDataLen(data_len);
	dataLenToBuffer(data_len);
}

uint32_t RpcConnection::convertRequestCmdType(MessageType cmd_type) // big to littile
{
	if (is_concordance_) return cmd_type;

	uint32_t type = 0;
	type = bigToLittile32(static_cast<uint32_t>(cmd_type));
	return static_cast<MessageType>(type);
}

uint32_t RpcConnection::convertResponseCmdType(MessageType cmd_type)
{
	if (is_concordance_) 
	{
		return static_cast<uint32_t>(cmd_type);
	}

	uint32_t type = 0;
	type = littileToBig32(static_cast<uint32_t>(cmd_type));
	return static_cast<MessageType>(type);
}

uint16_t RpcConnection::convertRequestDataLen(uint16_t data_len) // big to littile
{
	if (is_concordance_) return data_len;

	uint32_t len = 0;
	len = bigToLittile16(data_len);
	return len;
}

uint16_t RpcConnection::convertResponseDataLen(uint16_t data_len) // 
{
	if (is_concordance_) return data_len;

	uint32_t len = 0;
	len = littileToBig16(data_len);
	return len;
}

void RpcConnection::getTime()
{
#if 0
	SYSTEMTIME sys;
	GetLocalTime(&sys);
	printf(" %02d:%02d:%02d:%03d: ", sys.wHour, sys.wMinute, sys.wSecond, sys.wMilliseconds);
#endif
}


void RpcConnection::printResponseStr(uint32_t cmd_type, std::string data)
{
#if 0
	getTime();
	cout << "Response " << cmd_type << ": " << data << endl;
	printf("Response: %d: %s\n", cmd_type, data);
#endif
}

std::string RpcConnection::processResultStr(bool is_success)
{
	if (is_success)
	{
		return "Success";
	}

	return "Operation failed!";
}