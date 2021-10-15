#ifndef CONTROL_DASHBOARD_H
#define CONTROL_DASHBOARD_H

#include <string>
#include <vector>

#include "dashboard_comm.h"


class ControlDashboard
{
public:
    ControlDashboard(std::string host);
    ~ControlDashboard();

    bool start();
    void halt();
    bool isConneted();

    bool quit();
    bool getScopeVersion(std::string& version);
    bool getRobotMode(std::string& robot_mode);
    bool getSafetyMode(std::string& safety_mode);
    bool getProgramState(std::string& pro_mode);

    bool powerOn();
    void powerOff();
    bool brakeRelease();
    bool shutDown();
    bool isRunning();

    bool loadProgram(std::string ur_file);
    bool playProgram();
    bool stopProgram();
    bool pauseProgram();
    bool isProgramSaved();

    bool startPopUp(std::string show_msg); 
    bool closePopUp();

    bool unlockProtectStop(std::string& recv_msg); // not to open

private:
    std::string host_;
    DashboardComm* dashboard_comm_;

    enum CmdIndex { 
        POLY_SCOPE_VERSION,
        QUIT,
        ROBOT_MODE,
        SAFETY_MODE,
        PROGRAM_STATE
    }; 

   enum CheckCmdIndex { 
        POWER_ON,
        POWER_OFF, 
        BREAK_RELEASE,
        SHUT_DOWN,
        RUNNING,
        LOAD_PTOGRAM,
        PLAY_PTOGRAM,
        STOP_PTOGRAM,
        PAUSE_PTOGRAM,
        IS_PROGRAM_SAVED,
        START_POP_UP,
        CLOSE_POP_UP,
        CLOSE_SAFETY_POP_UP,
        UNLOCK_PROTECTIVE_STOP,
        RESTART_SAFETY,
        ADD_TO_LOG
    }; 

    std::vector<std::string> cmds_;
    std::vector<std::vector<std::string>> check_cmds_;

    void generateCmds();
    void generateCheckCmds();

    bool closePopUpSafety(std::string& recv_msg); // not to open

    bool restartSafety(std::string& recv_msg); // not to open
    bool addLog(std::string log); // not to open
};

#endif