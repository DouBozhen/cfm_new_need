#ifndef THREAD_BASE_H
#define THREAD_BASE_H

#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <pthread.h>

template <typename T>

class ThreadBase
{
public:
    ThreadBase();
    bool start(int priority);
    void stop();
    static void* threadProc(void* param);

protected:
    pthread_t thread_;
    bool is_running_;
    int priority_;
};


template <typename T>
ThreadBase<T>::ThreadBase()
{
    is_running_ = false;
    priority_ = 0;
}

template <typename T>
bool ThreadBase<T>::start(int priority)
{
    T* lhs = static_cast<T*>(this);

    int ret = pthread_create(&thread_, nullptr, threadProc, (void*)lhs);
    if(ret < 0)
    {
        printf("ThreadBase: create thread failed, %d\n", ret);
        return false;
    }
    printf("ThreadBase: create thread successful\n");

    is_running_ = true;
    return true;
}

template <typename T>
void ThreadBase<T>::stop()
{
    if(!is_running_) return;

    is_running_ = false;
    void* res;
    pthread_join(thread_, &res);
}

template <typename T>
void* ThreadBase<T>::threadProc(void* param)
{
    printf("ThreadBase: threadProc: to run\n");
     T* lhs = static_cast<T*>(param);
     lhs->run();
     return nullptr;
}

#endif

