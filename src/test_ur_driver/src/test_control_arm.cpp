#include <unistd.h>
#include "test_control_arm.h"

TestControlArm::TestControlArm(std::string ip)
{
    control_arm_ = new ControlArm(ip);
}

TestControlArm::~TestControlArm()
{
    if(control_arm_ != nullptr)
    {
        delete control_arm_;
        control_arm_ = nullptr;
    }
}

void TestControlArm::testGetParams()
{
    if(!control_arm_->start())
    {
        printf("URDriverSoftware: control arm start failed.\n");
        return;      
    }

    if(!control_arm_->isConnected())
    {
        printf("URDriverSoftware: control arm disconnected.\n");
        return;
    }

    sleep(3);

    int i = 0;
    while(i < 10)
    {
        Joint actual_joint = control_arm_->getActualJoint();
        Joint target_joint = control_arm_->getTargetJoint();

        printf("%d: actual_joint: %lf, %lf, %lf, %lf, %lf, %lf\n",i, 
            actual_joint.jVal[0],
            actual_joint.jVal[1],
            actual_joint.jVal[2],
            actual_joint.jVal[3],
            actual_joint.jVal[4],
            actual_joint.jVal[5]
            );

        printf("%d: target_joint: %lf, %lf, %lf, %lf, %lf, %lf\n", i,
            target_joint.jVal[0],
            target_joint.jVal[1],
            target_joint.jVal[2],
            target_joint.jVal[3],
            target_joint.jVal[4],
            target_joint.jVal[5]
            );
         sleep(1);
        i++;
    }

    control_arm_->halt();
}

