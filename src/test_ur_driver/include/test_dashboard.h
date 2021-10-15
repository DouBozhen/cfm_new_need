#ifndef TEST_CONTROL_DASHBOARD_H
#define TEST_CONTROL_DASHBOARD_H

#include <string>
#include "control_dashboard.h"

class TestControlDashboard
{
public:
    TestControlDashboard(std::string ip);
    ~TestControlDashboard();

    void testGetParams();

private:
    ControlDashboard* control_db_;
};

#endif