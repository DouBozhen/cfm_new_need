#include <errno.h>  
#include <unistd.h>
#include <cstring>
#include "tcp_server.h"

TcpServer::TcpServer()
{
	max_fd_ = -1;
	conn_list_lock_ = PTHREAD_MUTEX_INITIALIZER;
	memset(client_fds_, 0, sizeof(int) * CONCURRENT_MAX);

	initMember();
}

TcpServer::~TcpServer()
{
	is_running_ = false;

    close(socket_);
    pthread_mutex_destroy(&conn_list_lock_);
}

void TcpServer::initMember()
{
	socket_ = -1;
	is_running_ = false;
    port_ = 0;
	conn_list_.clear();
}

bool TcpServer::initServer(int port)
{
	socket_ = socket(AF_INET, SOCK_STREAM, 0);
	if (socket_ < 0)
	{
		cout << "TcpServer： create socket failed!" << endl;
		return false;
	}

    int flags = fcntl(socket_, F_GETFL, 0); //获取文件的flags值。
	fcntl(socket_, F_SETFL, flags | O_NONBLOCK);   //设置成非阻塞模式；

	struct sockaddr_in soc_addr;
    bzero(&soc_addr,sizeof(soc_addr));
    soc_addr.sin_family = AF_INET;
    soc_addr.sin_port = htons(port);
    soc_addr.sin_addr.s_addr = INADDR_ANY;

	int result = bind(socket_, (struct sockaddr*)&soc_addr, sizeof(sockaddr_in));
	
	if (result < 0)
	{
        printf("TcpServer: bind failed, %s(errno: %d)", strerror(errno), errno); 
   		return false;
	}

	result = listen(socket_, SOMAXCONN);
	if (result < 0)
	{
        printf("TcpServer: listen failed, %s(errno: %d)", strerror(errno), errno); 
   		return false;
	}
    port_ = port;
	return true;
}

bool TcpServer::isRunning()
{
	return is_running_;
}

bool TcpServer::startService()
{
	if (createHelperAndAcceptThread())
	{
		cout << "Start service success" << endl;
	}
	else
	{
		cout << "Start service failed" << endl;
		return false;
	}
	return true;
}

bool TcpServer::createHelperAndAcceptThread()
{
	is_running_ = true;
	unsigned long ul_thread_id = 0;
	int ret = pthread_create(&thread_help_, nullptr, helperThread, (void*)this);
	if (ret < 0)
	{
		is_running_ = false;
		return false;
	}

	ret = pthread_create(&thread_accept_, nullptr, acceptThread, (void*)this);
	if (ret < 0)
	{
		is_running_ = false;
		return false;
	}

	return true;
}

void TcpServer::stopService()
{
	bool result = true;

	is_running_ = false;
  
	void* state;
	pthread_join(thread_accept_, &state);
	pthread_join(thread_help_, &state);
}

void* TcpServer::acceptThread(void* param)
{
	TcpServer* tcp_server = static_cast<TcpServer*>(param);

	struct timeval timeout = {1,0};

	while (tcp_server->is_running_)
	{
		FD_ZERO(&tcp_server->server_fd_set_);

		FD_SET(STDIN_FILENO, &tcp_server->server_fd_set_);
		if(tcp_server->max_fd_ < STDIN_FILENO)
		{
			tcp_server->max_fd_ = STDIN_FILENO;
		}

		FD_SET(tcp_server->socket_, &tcp_server->server_fd_set_);
 		if(tcp_server->max_fd_ < tcp_server->socket_)
        {
        	tcp_server->max_fd_ = tcp_server->socket_;
        }

		for(int i = 0; i < CONCURRENT_MAX; i++)
        {
        	//printf("client_fds[%d]=%d\n", i, client_fds[i]);
        	if(tcp_server->client_fds_[i] != 0)
        	{
        		FD_SET(tcp_server->client_fds_[i], &tcp_server->server_fd_set_);
        		if(tcp_server->max_fd_ < tcp_server->client_fds_[i])
        		{
        			tcp_server->max_fd_ = tcp_server->client_fds_[i];
        		}
        	}
        }

		int ret = select(tcp_server->max_fd_ + 1, &tcp_server->server_fd_set_, NULL, NULL, &timeout);

      	if(ret < 0)
        {
        	perror("select 出错\n");
            usleep(TIMEOUT_CONN);
        	continue;
        }
        else if(ret == 0)
        {
            usleep(TIMEOUT_CONN);
        	continue;
        }
        else
		{
			if(FD_ISSET(tcp_server->socket_, &tcp_server->server_fd_set_))
        	{
				struct sockaddr_in conn_addr;
				socklen_t address_len;

				int conn_sock_fd = accept(tcp_server->socket_, (struct sockaddr *)&conn_addr, &address_len);
				printf("new connection conn_sock_fd = %d\n", conn_sock_fd);

				if(conn_sock_fd > 0)
				{
					int index = -1;
        			for(int i = 0; i < CONCURRENT_MAX; i++)
        			{
        				if(tcp_server->client_fds_[i] == 0)
        				{
        					index = i;
        					tcp_server->client_fds_[i] = conn_sock_fd;
        					break;
        				}
        			}

					if(index >= 0)
        			{
        				printf("New client (%d) joined successfully %s:%d\n", index, inet_ntoa(conn_addr.sin_addr), ntohs(conn_addr.sin_port));

						Connection* pconn = nullptr;
						if (tcp_server->port_ == RPC_PORT)
						{
							pconn = new RpcConnection(conn_sock_fd, tcp_server->server_fd_set_);
						}
						else if (tcp_server->port_ == PUB_PORT)
						{
							pconn = new PubConnection(conn_sock_fd, tcp_server->server_fd_set_);
						}

						if (pconn != nullptr)
						{
							pthread_mutex_lock(&tcp_server->conn_list_lock_);
							tcp_server->conn_list_.push_back(pconn);
							tcp_server->conn_list_.back()->startRunning();
							pthread_mutex_unlock(&tcp_server->conn_list_lock_);
						}
						if (pconn != nullptr)
						{
							// delete pconn;
						}
						
						//pconn->startRunning();
        			}
        			else
        			{
        				printf("The number of client connections has reached the maximum, and the new client failed to join %s:%d\n", inet_ntoa(conn_addr.sin_addr), ntohs(conn_addr.sin_port));
        			}
				}

			}

		}
		usleep(TIMEOUT_CONN);
	}
	return 0;
}

void* TcpServer::helperThread(void* param)
{
	TcpServer* tcp_server = static_cast<TcpServer*>(param);
	while (tcp_server->is_running_)
	{
		pthread_mutex_lock(&tcp_server->conn_list_lock_);
	
		std::list<Connection*>::iterator it;
		for (it = tcp_server->conn_list_.begin(); it != tcp_server->conn_list_.end();)
		{
		    Connection* connection = (Connection*)*it;
			int conn_socket = connection->getConnSocket();
			if (connection->isExit())
			{
				for(int i = 0; i < CONCURRENT_MAX; i++)
				{
					if(tcp_server->client_fds_[i] == conn_socket)
					{
						FD_CLR(tcp_server->client_fds_[i], &tcp_server->server_fd_set_);
						tcp_server->client_fds_[i] = 0;
					}
				}

				tcp_server->conn_list_.erase(it++);
				delete connection;
				connection = nullptr;
				printf("connection removed\n");
			}
			else
			{
				it++;
			}
		}

		pthread_mutex_unlock(&tcp_server->conn_list_lock_);
		usleep(TIMEOUT_CONN);
	}

	if (tcp_server->is_running_)
	{
		pthread_mutex_lock(&tcp_server->conn_list_lock_);
		std::list<Connection*>::iterator it;
		for (it = tcp_server->conn_list_.begin(); it != tcp_server->conn_list_.end();)
		{
			Connection* connection = (Connection*)*it;
			if (connection->isConning())
			{
				connection->disConning();
			}
			it++;

			usleep(TIMEOUT_CONN / 100);
		}
		pthread_mutex_unlock(&tcp_server->conn_list_lock_);

		usleep(TIMEOUT_CONN);

		pthread_mutex_lock(&tcp_server->conn_list_lock_);
		while (tcp_server->conn_list_.size() != 0)
		{
			for (it = tcp_server->conn_list_.begin(); it != tcp_server->conn_list_.end();)
			{
				Connection* connection = (Connection*)*it;
				int conn_socket = connection->getConnSocket();
				if (connection->isExit())
				{
					for(int i = 0; i < CONCURRENT_MAX; i++)
					{
						if(tcp_server->client_fds_[i] == conn_socket)
						{
							FD_CLR(tcp_server->client_fds_[i], &tcp_server->server_fd_set_);
							tcp_server->client_fds_[i] = 0;
						}
					}

					tcp_server->conn_list_.erase(it++);
					delete connection;
					connection = nullptr;
					printf("connection removed\n");
				}
				else
				{
					it++;
				}
			}

			usleep(TIMEOUT_CONN);
		}
		pthread_mutex_unlock(&tcp_server->conn_list_lock_);
	}

	tcp_server->conn_list_.clear();
	// SetEvent(tcp_server->thread_help_);

	return 0;
}

