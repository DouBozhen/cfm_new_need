#ifndef CONNECTION_H
#define CONNECTION_H

constexpr auto RPC_PORT = 30800;
constexpr auto PUB_PORT = 30803;

constexpr auto THREAD_TIMEOUT = 10000; //10ms
constexpr auto RECV_REPEATED_COUNT = 5;
constexpr auto SEND_REPEATED_COUNT = 5;

#define CONCURRENT_MAX 8

class Connection 
{
public:
	virtual bool startRunning() = 0;
	virtual void disConning()  = 0;

	virtual bool isConning()  = 0;
	virtual bool isExit()  = 0;

	virtual int getConnSocket() = 0;
};

#endif
