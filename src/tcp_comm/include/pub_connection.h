#ifndef PUB_CONNECTION_H
#define PUB_CONNECTION_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <fcntl.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <mutex>

#include "connection.h"
#include "force_master_pub.pb.h"

using namespace std;

class PubConnection : public Connection
{
public:
    PubConnection(int socket, fd_set server_fd_set);
    ~PubConnection();

	virtual bool startRunning() override;
	virtual void disConning() override;

	static void* pubMessageThread(void* param);

	virtual bool isConning() override;
	virtual bool isExit() override;
	virtual int getConnSocket() override;
private:
	enum {
		DATA_BUFFER_LEN = 1024, HEADER_LEN = 4,
	};

    int s_;
	int conn_fd_[CONCURRENT_MAX];

    char* pub_buffer_;
	int pub_buffer_size_;
    int pub_times_;

    pthread_mutex_t mutex_;

    bool is_conning_;
	bool is_exit_;
    pthread_t pub_thread_;

    bool is_concordance_;

    typedef void (PubConnection::*HandleFuncPtr)(int);
	typedef struct
	{
		//ForceMasterPub::MessageType cmd_type;
		HandleFuncPtr request_func_ptr;
	}PubService;

	std::vector<PubService> pub_table_;
	pthread_mutex_t rpc_table_mutex_;

	void initPubTable();
	void handlePubMessage(char* buffer, int &data_len);

	PubConnection();

    void processGetJointPosition(int data_size){}
};

#endif

