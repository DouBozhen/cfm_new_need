#ifndef TEST_CONTROL_ARM_RT_H
#define TEST_CONTROL_ARM_RT_H

#include <string>
#include <vector>
#include "control_arm_rt.h"

class TestControlArmRT
{
public:
    TestControlArmRT(std::string ip);
    ~TestControlArmRT();

    void testGetParams();
    bool testSendScript();
private:
    ControlArmRealtime* control_arm_rt_;
};

#endif