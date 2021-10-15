
#include "control_dashboard.h"


ControlDashboard::ControlDashboard(std::string host) : host_(host)
{
    cmds_.clear();
    check_cmds_.clear();

    generateCmds();
    generateCheckCmds();
    dashboard_comm_ = new DashboardComm(host);
}

ControlDashboard::~ControlDashboard()
{
    if(dashboard_comm_ != nullptr)
    {
        delete dashboard_comm_;
        dashboard_comm_ = nullptr;
    }

    cmds_.clear();
    check_cmds_.clear();
}

bool ControlDashboard::start()
{
    return dashboard_comm_->start();
}

void ControlDashboard::halt()
{
    dashboard_comm_->halt();  
}

bool ControlDashboard::isConneted()
{
    dashboard_comm_->isConneted();  
}
