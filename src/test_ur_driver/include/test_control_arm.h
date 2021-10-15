#ifndef TEST_CONTROL_ARM_H
#define TEST_CONTROL_ARM_H

#include <string>
#include <vector>
#include "control_arm.h"

class TestControlArm
{
public:
    TestControlArm(std::string ip);
    ~TestControlArm();

    void testGetParams();

private:
    ControlArm* control_arm_;
};

#endif