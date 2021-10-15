#include <unistd.h>
#include "test_dashboard.h"
using namespace std;

TestControlDashboard::TestControlDashboard(std::string ip)
{
    control_db_ = new ControlDashboard(ip);
    printf("create control db ok, ip: %s\n", ip.c_str());
}
TestControlDashboard::~TestControlDashboard()
{
    if(control_db_ != nullptr)
    {
        delete control_db_;
        control_db_ = nullptr;
    }
}

void TestControlDashboard::testGetParams()
{
    control_db_->start();
    sleep(3);

    if(!control_db_->isConneted())
    {
        printf("URDriverSoftware: dashboard disconnected.\n");
        return;
    }

    string version;
    control_db_->getScopeVersion(version);

    string robot_mode;
    control_db_->getRobotMode(robot_mode);

    control_db_->halt();
}
