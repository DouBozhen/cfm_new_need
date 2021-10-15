#include <unistd.h>
#include "controller.h"
#include "ur_datatype.h"

using namespace ur_data_type;

void Controller::shutDown()
{
    control_db_->shutDown();
}

void Controller::powerOff()
{
    control_db_->powerOff();
}

bool Controller::startRobot()
{
    printf("Controller: DashBoard start......\n");
    if(!control_db_->start())
    {
        printf("Controller: start dash board failed.\n");
        return false;  
    }
    printf("Controller: DashBoard start successful.\n");
    if(!control_db_->powerOn())
    {
        printf("Controller: DashBoard power on failed.\n");
        return false;  
    }
    printf("Controller: DashBoard power on successful.\n");
    if(!control_db_->brakeRelease())
    {
        printf("Controller: DashBoard brake release failed.\n");
        return false;  
    }
    usleep(5000000);
    printf("Controller: DashBoard brake realse successful.\n");

    control_db_->closePopUp();

    string recv_msg;
    if(!control_db_->unlockProtectStop(recv_msg))
    {
        printf("Controller: DashBoard unlock protect stop failed.\n");
        return false;  
    }
    printf("Controller: DashBoard %s.\n", recv_msg.c_str());

    if(!control_arm_rt_->start())
    {
        printf("Controller: start robot arm controller.failed.\n");
        return false;  
    }
    printf("Controller: start robot arm controller successful.\n");
    
    usleep(1000000);
    
    printf("Controller: start robot ......");
    int i = 0;
    for (i = 0; i != ROBOT_COMM_COUNT; ++i)
    {
        Joint actual_joint = control_arm_rt_->getActualJoint();
        if(actual_joint.jVal[0] < DOUBLE_ZERO
            && actual_joint.jVal[1] < DOUBLE_ZERO
            && actual_joint.jVal[2] < DOUBLE_ZERO
            && actual_joint.jVal[3] < DOUBLE_ZERO
            && actual_joint.jVal[4] < DOUBLE_ZERO
            && actual_joint.jVal[5] < DOUBLE_ZERO
            )
        {
            //printf("Controller: start failed, control_arm_rt point error.\n");
            usleep(100000); 
            printf(".");
            continue;
        }
        else
        {
            break;
        }
    }
    
    if (i >= ROBOT_COMM_COUNT)
    {
        printf("Controller: start failed, control_arm_rt point error.\n");
        return false;
    }   

    control_arm_rt_->initGlobalSpeed();
    printf("UR Robot Control version: %s\n", control_arm_rt_->getVersion().c_str());
    return true;
}

void Controller::stopRobot()
{
    control_db_->halt();
}

