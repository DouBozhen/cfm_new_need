#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include "controller.h"
#include "app_game_model.h"
#include "training_manual_move.h"

using namespace app_game_model;

#define START_APP_GAME

Controller* g_controller = nullptr;

void onExit(int signal_n)
{
    printf("on exiting controller...\n");
#ifdef START_APP_GAME
//    g_controller->dropGame();
#endif
    printf("stopGame successful.\n");

    g_controller->stop();
    printf("stop controller successful.\n");
}

int main(int argc, char** argv)
{
    g_controller = Controller::getInstance();
    printf("*************** init controller *************\n");
    if (!g_controller->init())
    {
        printf("init controller failed.\n");
        return false;
    }
    printf("init controller successful.\n");

    printf("*************** start controller *************\n");
    if (!g_controller->start())
    {
        printf("start controller failed.\n");
        return false;
    }
    printf("start controller successful.\n");
    sleep(1);
    AppGameParamsSub params;
    params.force_level = FORCE_LEVEL_SMALL;
    params.rigidity_level = RIGIDITY_SMALL;
    params.speed_level = SPEED_LEVEL_SLOW;//FAST
    params.repeated_count = 3;

#ifdef START_APP_GAME
    printf("*************** start game *************\n");
    if(!g_controller->createGame(MANUAL_MOVEMENT, params))//TRAINING_MANUAL_MOVE??
    {
        printf("createGame failed.\n");
        return false;  
    }
    printf("createGame successful.\n");

    if(!g_controller->startGame())
    {
        printf("start failed.\n");
        return false;  
    }
    printf("startGame successful.\n");
#endif
    while (g_controller->isRunning())
    {
        signal(SIGINT, onExit);
        signal(SIGTERM, onExit);
        signal(SIGSEGV, onExit);
        usleep(50000);
    }

}


