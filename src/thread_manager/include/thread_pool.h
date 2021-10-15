#ifndef THREAD_POOL_H
#define THREAD_POOL_H

#include <pthread.h>
#include <mutex>
#include "handler.h"
#include "thread_base.h"
#include "thread_work.h"

#define MAX_THREAD_COUNT 2

class ThreadPool
{
public:
    ThreadPool();
    ~ThreadPool();
    static ThreadPool& getInstance();

    bool registerHandler(Handler* handler, int index);
    void deregisterHandler(int index);

    void startAllocatePool();
    void stopAllocatePool();

    void notifyCall();

private:
    std::mutex lock_;

    Handler* handlers_[MAX_THREAD_COUNT];
    ThreadWork* threads_[MAX_THREAD_COUNT];
    static std::mutex mutex_;
};

#endif
