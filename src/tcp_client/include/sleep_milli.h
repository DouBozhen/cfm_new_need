#ifndef SLEEP_MILLI_H 
#define SLEEP_MILLI_H

#include <sys/time.h>

static void sleep_milli(int milli_sec)
{
    struct timeval sTime;
    sTime.tv_sec    = 0;
    sTime.tv_usec   = milli_sec * 1000;
    select(0, NULL, NULL, NULL, &sTime);
}

#endif