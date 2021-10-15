#ifndef TCP_SERVER_MANAGER_H
#define TCP_SERVER_MANAGER_H

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <string>
#include <stdio.h>
#include <vector>
#include <iostream>
#include <mutex>
#include <list>
#include <mutex>

#include "tcp_server.h"

class TcpServerManager
{
public:
	static TcpServerManager* getInstance();
	~TcpServerManager();

	bool initServer();
	bool startService();
	void stopService();
	bool isRunning();
private:
	bool is_running_;
	static TcpServerManager* instance_;

	TcpServer* rpc_server_;
	TcpServer* pub_server_;

	TcpServerManager();
};

#endif
