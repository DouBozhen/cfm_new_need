#include <string>
#include "control_dashboard.h"

using namespace std;

bool ControlDashboard::getScopeVersion(std::string& version)
{
    return dashboard_comm_->processCommand(cmds_[POLY_SCOPE_VERSION], version);
}

bool ControlDashboard::quit()
{
    string recv_msg;
    return dashboard_comm_->processCommand(cmds_[QUIT], recv_msg);
}

bool ControlDashboard::getRobotMode(std::string& robot_mode)
{
     return dashboard_comm_->processCommand(cmds_[ROBOT_MODE], robot_mode);
}

bool ControlDashboard::getSafetyMode(std::string& safety_mode)
{
     return dashboard_comm_->processCommand(cmds_[SAFETY_MODE], safety_mode);
}

bool ControlDashboard::getProgramState(std::string& pro_mode)
{
     return dashboard_comm_->processCommand(cmds_[PROGRAM_STATE], pro_mode);
}

/*process check cmds*/
bool ControlDashboard::powerOn()
{
    std::string recv_msg;
     bool result = dashboard_comm_->processCommand(
         check_cmds_[POWER_ON][0], 
         check_cmds_[POWER_ON][1], 
         recv_msg);
    if(!result)
    {
        printf("ControlDashboard: shut down failed, %s.", recv_msg.c_str());
        return false;
    }

    return true;
}

void ControlDashboard::powerOff()
{
     std::string recv_msg;
     dashboard_comm_->processCommand(
         check_cmds_[POWER_OFF][0], 
         check_cmds_[POWER_OFF][1], 
         recv_msg);
}

bool ControlDashboard::brakeRelease()
{
     std::string recv_msg;
     bool result = dashboard_comm_->processCommand(
         check_cmds_[BREAK_RELEASE][0], 
         check_cmds_[BREAK_RELEASE][1], 
         recv_msg);
    if(!result)
    {
        printf("ControlDashboard: shut down failed, %s.", recv_msg.c_str());
        return false;
    }

    return true;
}

bool ControlDashboard::shutDown()
{
     std::string recv_msg;
     bool result = dashboard_comm_->processCommand(
         check_cmds_[SHUT_DOWN][0], 
         check_cmds_[SHUT_DOWN][1], 
         recv_msg);
    if(!result)
    {
        printf("ControlDashboard: shut down failed, %s.", recv_msg.c_str());
        return false;
    }

    return true;
}

bool ControlDashboard::isRunning()
{
     string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[RUNNING][0], 
         check_cmds_[RUNNING][1], 
         recv_msg);
}

bool ControlDashboard::loadProgram(std::string ur_file)
{
    string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[LOAD_PTOGRAM][0] + ur_file, 
         check_cmds_[LOAD_PTOGRAM][1], 
         recv_msg);
}

bool ControlDashboard::playProgram()
{
    string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[PLAY_PTOGRAM][0], 
         check_cmds_[PLAY_PTOGRAM][1], 
         recv_msg);
}

bool ControlDashboard::stopProgram()
{
    string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[STOP_PTOGRAM][0], 
         check_cmds_[STOP_PTOGRAM][1], 
         recv_msg);
}

bool ControlDashboard::pauseProgram()
{
    string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[PAUSE_PTOGRAM][0], 
         check_cmds_[PAUSE_PTOGRAM][1], 
         recv_msg);
}

bool ControlDashboard::isProgramSaved()
{
    string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[IS_PROGRAM_SAVED][0], 
         check_cmds_[IS_PROGRAM_SAVED][1], 
         recv_msg);
}

bool ControlDashboard::startPopUp(std::string show_msg)
{
    string recv_msg;
    return dashboard_comm_->processCommand(
        check_cmds_[START_POP_UP][0] + show_msg, 
        check_cmds_[START_POP_UP][1], 
        recv_msg);
}

bool ControlDashboard::closePopUp()
{
     std::string recv_msg;
     bool result = dashboard_comm_->processCommand(
         check_cmds_[CLOSE_POP_UP][0], 
         check_cmds_[CLOSE_POP_UP][1], 
         recv_msg);
    if(!result)
    {
        printf("ControlDashboard: shut down failed, %s.", recv_msg.c_str());
        return false;
    }

    return true;
}

bool ControlDashboard::closePopUpSafety(std::string& recv_msg)
{
     return dashboard_comm_->processCommand(
         check_cmds_[CLOSE_SAFETY_POP_UP][0], 
         check_cmds_[CLOSE_SAFETY_POP_UP][1], 
         recv_msg);
}

bool ControlDashboard::unlockProtectStop(std::string& recv_msg)
{
     return dashboard_comm_->processCommand(
         check_cmds_[UNLOCK_PROTECTIVE_STOP][0], 
         check_cmds_[UNLOCK_PROTECTIVE_STOP][1], 
         recv_msg);
}

bool ControlDashboard::restartSafety(std::string& recv_msg)
{
     return dashboard_comm_->processCommand(
         check_cmds_[RESTART_SAFETY][0], 
         check_cmds_[RESTART_SAFETY][1], 
         recv_msg);
}

bool ControlDashboard::addLog(std::string log)
{
    string recv_msg;
     return dashboard_comm_->processCommand(
         check_cmds_[ADD_TO_LOG][0] + log, 
         check_cmds_[ADD_TO_LOG][1], 
         recv_msg);
}
