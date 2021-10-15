#ifndef TCP_SERVER_H
#define TCP_SERVER_H

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
#include <list>
#include <mutex> 

#include "pub_connection.h"
#include "rpc_connection.h"
#include "connection.h"
#include "tcp_server.h"

// constexpr auto SERVER_PORT = 30800;
constexpr auto TIMEOUT_THREAD_EXIT = 5000; // mian thread sleep time
constexpr auto TIMEOUT_CONN = 1000000; // 1s
constexpr auto TIMEOUT_THREAD_HELPER = 500;

class TcpServer
{
public:
	TcpServer();
	~TcpServer();

	bool initServer(int port);
	bool startService();
	void stopService();
	bool isRunning();
private:
	bool is_running_;
	int socket_;
    int port_;
	int client_fds_[CONCURRENT_MAX];
	fd_set server_fd_set_;
    int max_fd_;

	pthread_t thread_accept_;
	pthread_t thread_help_;
	std::list<Connection*> conn_list_;
	pthread_mutex_t conn_list_lock_;

	void initMember(); 

	bool createHelperAndAcceptThread();
	static void* acceptThread(void* param);
	static void* helperThread(void* param);
};

#endif
