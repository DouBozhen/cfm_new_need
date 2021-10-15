#include <errno.h>  
#include <unistd.h>
#include "tcp_server_manager.h"

TcpServerManager* TcpServerManager::instance_ = NULL;

TcpServerManager::TcpServerManager()
{
	rpc_server_ = new TcpServer();
	pub_server_ = new TcpServer();
}

TcpServerManager::~TcpServerManager()
{
	if (rpc_server_ != nullptr)
	{
		delete rpc_server_;
		rpc_server_ = nullptr;
	}
	if (pub_server_ != nullptr)
	{
		delete pub_server_;
		pub_server_ = nullptr;
	}
	if (instance_ != nullptr)
	{
		delete instance_;
		instance_ = nullptr;
	}	
}

TcpServerManager* TcpServerManager::getInstance()
{
	if (instance_ == nullptr)
	{
		instance_ = new TcpServerManager();
	}

	return instance_;
}

bool TcpServerManager::initServer()
{
	if (!rpc_server_->initServer(RPC_PORT))
	{
		printf("TcpServerManager: init rpc server failed.\n");
		return false;
	}
#if 0
	if (!pub_server_->initServer(PUB_PORT))
	{
		rpc_server_->stopService();
		printf("TcpServerManager: init pub server failed.\n");
		return false;
	}
#endif
	return true;
}

bool TcpServerManager::isRunning()
{
	if (rpc_server_->isRunning() && pub_server_->isRunning())
	{
		return true;
	}
	return false;
}

bool TcpServerManager::startService()
{
	if (!rpc_server_->startService())
	{
		printf("TcpServerManager: init start service failed.\n");
		return false;
	}
#if 0
	if (!pub_server_->startService())
	{
		rpc_server_->stopService();
		printf("TcpServerManager: init start service failed.\n");
		return false;
	}
#endif
	return true;
}

void TcpServerManager::stopService()
{
	rpc_server_->stopService();
	// pub_server_->stopService();
}
