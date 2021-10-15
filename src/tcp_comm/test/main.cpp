#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <mutex>

#ifdef ROS_LAUNCH
	#include <ros/ros.h>
#endif

#include "controller.h"
#include "app_game_model.h"
#include "tcp_server_manager.h"

using namespace app_game_model;

Controller* g_controller = nullptr;
TcpServerManager* g_tcp_server = nullptr;
std::mutex g_lock;

void onExit(int signal_n)
{
    std::unique_lock<std::mutex> l{ g_lock };
#if 1
    if (g_tcp_server != nullptr && g_tcp_server->isRunning())
    {
        g_tcp_server->stopService();
    }
    usleep(1000000);
    printf("stop servers successful.\n");
#endif

    if (g_controller != nullptr && g_controller->isRunning())
    {
        g_controller->stop();
    }
    usleep(1000000);
    printf("stop controller successful.\n");
}

int main(int argc, char** argv)
{
    sigset_t set;
    sigemptyset(&set);
    sigaddset(&set, SIGPIPE);
    sigprocmask(SIG_BLOCK, &set, NULL);

    signal(SIGCHLD, SIG_IGN);
#if 1
    g_controller = Controller::getInstance();
    if (!g_controller->init())
    {
        printf("init controller failed.\n");
        return -1;
    }

    printf("init controller successful.\n");
    //sleep(1);
    
    if (!g_controller->start())
    {
        printf("start controller failed.\n");
        return -1;
    }

    printf("start controller successful.\n");
    sleep(1);
#endif
#if 1
    g_tcp_server = TcpServerManager::getInstance();
    if (!g_tcp_server->initServer())
    {
        printf("init servers failed.\n");
        return -1;
    }
    printf("init servers successful.\n");

    if (!g_tcp_server->startService())
    {
        printf("start servers failed.\n");
        return -1;
    }
    printf("start servers successful.\n");
#endif

#ifdef ROS_LAUNCH
    ros::init(argc, argv, "tcp_comm_node") ;
    ros::NodeHandle nh;
    ROS_INFO_STREAM("tcp_comm_node running.") ;
#endif
    while (g_controller->isRunning())
    {
        signal(SIGINT, onExit);
        signal(SIGTERM, onExit);
        signal(SIGSEGV, onExit);
        
        usleep(1000000);
    }
   
    return 0;
}
