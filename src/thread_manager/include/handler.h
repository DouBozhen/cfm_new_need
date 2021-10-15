#ifndef HANDLER_H
#define HANDLER_H

#include <string>

class Handler
{
public: 
    virtual void process() = 0;
    virtual std::string getName() = 0;
};


#endif