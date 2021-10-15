#include <errno.h>  
#include <unistd.h>
#include <cstring>
#include "pub_connection.h"

using namespace ForceMasterPub;

PubConnection::PubConnection(int socket, fd_set server_fd_set)
{
    mutex_ = PTHREAD_MUTEX_INITIALIZER;

    s_ = socket;
	// conn_fd_ = conn_fds;

	is_conning_ = false;
	is_exit_ = false;

	is_concordance_ = true;

	pub_buffer_ = new char[DATA_BUFFER_LEN]();
	memset(pub_buffer_, 0, DATA_BUFFER_LEN);
    pub_times_ = 0;

	pub_table_.clear();
	initPubTable();
}

PubConnection::~PubConnection()
{
    close(s_);
    is_conning_ = false;

    void* res;
    pthread_join(pub_thread_, &res);
    pthread_mutex_destroy(&mutex_);
	pub_table_.clear();

    if (pub_buffer_ != nullptr)
	{
		delete[] pub_buffer_;
		pub_buffer_ = nullptr;
	}
}

int PubConnection::getConnSocket()
{
	return s_;
}

bool PubConnection::startRunning()
{
	is_conning_ = true;

	int ret = pthread_create(&pub_thread_, nullptr, pubMessageThread, (void*)this);
    if(ret < 0)
    {
        printf("PubConnection: create publish thread failed, %d\n", ret);
        return false;
    }
    printf("PubConnection: create publish thread successful\n");

	cout << "Socket is connecting to client." << endl;
	return true;
}

void PubConnection::disConning()
{
    is_conning_ = false;
}

bool PubConnection::isConning()
{
    return is_conning_;
}

bool PubConnection::isExit()
{
	return is_exit_;
}

void* PubConnection::pubMessageThread(void* param)
{
    PubConnection* conn = static_cast<PubConnection*>(param);

	while (conn->is_conning_)
	{
        printf("PubConnection: pubMessageThread\n");
        if (!conn->is_conning_)
        {
            conn->is_exit_ = true;
            break;
        }
        int send_len = HEADER_LEN;

        pthread_mutex_lock(&conn->mutex_);
        conn->handlePubMessage(&conn->pub_buffer_[HEADER_LEN], send_len);
		memcpy(conn->pub_buffer_, &send_len, sizeof(send_len));

        int bytes = send(conn->s_, (char*)conn->pub_buffer_, send_len, 0);
        if (bytes < 0)
        {
            pthread_mutex_unlock(&conn->mutex_);
            if (conn->pub_times_ < SEND_REPEATED_COUNT)
            {
                conn->pub_times_++;
                continue;
            }
            else 
            {
                conn->pub_times_ = 0;
                break;
            }
        }
        memset(conn->pub_buffer_, 0, conn->DATA_BUFFER_LEN);
        pthread_mutex_unlock(&conn->mutex_);

		usleep(THREAD_TIMEOUT);
	}

    conn->is_conning_ = false;
    conn->is_exit_ = true;

	return nullptr;
}

