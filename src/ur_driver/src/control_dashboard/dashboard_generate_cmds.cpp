
#include "control_dashboard.h"
using namespace std;


void ControlDashboard::generateCmds()
{
    cmds_.push_back("polyscopeVersion");
    cmds_.push_back("quit");
    cmds_.push_back("robotmode");
    cmds_.push_back("safetymode");
    cmds_.push_back("programState");
}

void ControlDashboard::generateCheckCmds()
{
    vector<string> cmd;

    cmd.push_back("power On");
    cmd.push_back("Powering on");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("power off");
    cmd.push_back("Powering off");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("brake release");
    cmd.push_back("Brake releasing");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("shutdown");
    cmd.push_back("Shutting down");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("running");
    cmd.push_back("true");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("load ");
    cmd.push_back("Loading program");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("play");
    cmd.push_back("Starting program");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("stop");
    cmd.push_back("Stopped");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("pause");
    cmd.push_back("Pausing program");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("isProgramSaved");
    cmd.push_back("true");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("popup <pppoooppp>");
    cmd.push_back("showing popup");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("close popup");
    cmd.push_back("closing popup");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("close safety popup");
    cmd.push_back("closing safety popup");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("unlock protective stop");
    cmd.push_back("Protective stop releasing");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("restart safety");
    cmd.push_back("	Restarting safety");
    check_cmds_.push_back(cmd);
    cmd.clear();

    cmd.push_back("addToLog <log-message>");
    cmd.push_back(" message");
    check_cmds_.push_back(cmd);
    cmd.clear();
}