#ifndef THREAD_WORK_H
#define THREAD_WORK_H

#include <pthread.h>
#include "handler.h"
#include "thread_base.h"


class ThreadWork : public ThreadBase<ThreadWork>
{
public:
    ThreadWork();
    bool start();
    void stop();
    void run();
    void notifyCall(Handler* handler);
    void setId(int id);

private:
    bool run_once_;
    Handler *volatile handler_;
    pthread_cond_t need_process_;
    pthread_mutex_t lock_;

    bool is_waiting_;
    int t_id_;
};



#endif