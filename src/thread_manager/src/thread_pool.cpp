
#include <mutex>
#include "thread_pool.h"

ThreadPool::ThreadPool()
{
	for (uint32_t i = 0; i < MAX_THREAD_COUNT; i++)
	{
		threads_[i] = new ThreadWork();
	}
}

ThreadPool::~ThreadPool()
{
	for (uint32_t i = 0; i < MAX_THREAD_COUNT; i++)
	{
		if (threads_[i] != nullptr)
		{
			delete threads_[i];
			threads_[i] = nullptr;
		}
	}
}

ThreadPool& ThreadPool::getInstance()
{
 	static ThreadPool thread_pool_instance_;
	return thread_pool_instance_;
}

bool ThreadPool::registerHandler(Handler* handler, int index)
{
    std::unique_lock<std::mutex> l{ lock_ };
	if (index > MAX_THREAD_COUNT - 1)
	{
		printf("ThreadPool: index out of range\n");
		return false;
	}
	handlers_[index] = handler;
	printf("ThreadPool: register %dth handler: %s\n", index, handlers_[index]->getName().c_str());
    return true;
}

void ThreadPool::deregisterHandler(int index)
{
	std::unique_lock<std::mutex> l{ lock_ };
	if (index > MAX_THREAD_COUNT - 1)
	{
		printf("ThreadPool: index out of range\n");
		return ;
	}
	printf("ThreadPool: remove %dth handler: %s\n", index, handlers_[index]->getName().c_str());
	handlers_[index] = nullptr;
}

void ThreadPool::startAllocatePool()
{
	std::unique_lock<std::mutex> l{ lock_ };

	for (uint32_t i = 0; i < MAX_THREAD_COUNT; i++)
	{
		threads_[i]->setId(i);
		threads_[i]->start();
	}
	printf("ThreadPool: total thread count: %d\n", MAX_THREAD_COUNT);
}

void ThreadPool::stopAllocatePool()
{
    for (int i = 0; i != MAX_THREAD_COUNT; ++i)
	{
		if (threads_[i] != nullptr)
		{
			threads_[i]->stop();
			printf("stoped %dth thread\n", i);
		}
	}
}

void ThreadPool::notifyCall()
{
	for (int i = 0; i < MAX_THREAD_COUNT; i++)
	{
		if (threads_[i] != nullptr)
		{
			threads_[i]->notifyCall(handlers_[i]);
		}
	}
}


