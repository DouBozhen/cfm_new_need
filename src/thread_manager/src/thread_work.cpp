#include <stdio.h>
#include <stdlib.h>
#include <mutex>
#include "thread_work.h"

ThreadWork::ThreadWork()
{
    run_once_ = false;
    need_process_ = PTHREAD_COND_INITIALIZER;
    lock_ = PTHREAD_MUTEX_INITIALIZER;
    is_waiting_ = false;
    t_id_ = -1;
}

bool ThreadWork::start()
{
   return ThreadBase<ThreadWork>::start(SCHED_RR);    
}

void ThreadWork::stop()
{
    is_waiting_ = false;
    is_running_ = false;
    handler_ = nullptr;
    notifyCall(handler_);

    ThreadBase<ThreadWork>::stop();
}

void ThreadWork::run()
{
    while(is_running_)
    {
        //pthread_mutex_lock(&lock_);
        is_waiting_ = true;
        pthread_cond_wait(&need_process_, &lock_);
        is_waiting_ = false;
        //pthread_mutex_unlock(&lock_);

        if(handler_ != nullptr)
        {
            handler_->process();
        }

        run_once_ = false;
    }
}

void ThreadWork::notifyCall(Handler* handler)
{
    if(handler == nullptr) 
    {
        return;
    }

    if(run_once_ && is_waiting_) return;

    handler_ = handler;
    run_once_ = true;

    pthread_cond_signal(&need_process_);
}

void ThreadWork::setId(int id)
{
    t_id_ = id;
}
