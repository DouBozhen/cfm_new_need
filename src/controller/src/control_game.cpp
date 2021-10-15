#include <unistd.h>
#include "controller.h"
#include "app_game_model.h"
#include "force_datatype.h"

using namespace app_game_model;

bool Controller::createGame(int game_mode, app_game_model::AppGameParamsSub sub_params)
{
    sub_params.sensor_ip = ft_sensor_ip_;
    return app_game_manage_.createGame(game_mode, sub_params);
}   

bool Controller::startGame()
{
    return app_game_manage_.startGame(control_arm_rt_, &ft_sensor_);
}

void Controller::stopGame()
{
    app_game_manage_.stopGame();
}

void Controller::dropGame()
{
    app_game_manage_.dropGame();
}

bool Controller::setGameForceLevel(int level)
{
    return app_game_manage_.setForceLevel(level);
}

bool Controller::getGameForceLevel(int &level)
{
    return app_game_manage_.getForceLevel(level);
}

bool Controller::setGameRigidityLevel(int level)
{
    return app_game_manage_.setRigidityLevel(level);
}

bool Controller::setGameRepeatedCount(int count)
{
    return app_game_manage_.setRepeatedCount(count);
}

bool Controller::setGameSpeedLevel(int level)
{
    return app_game_manage_.setSpeedLevel(level);
}

bool Controller::getGameRepeatedCount(int &count)
{
    return app_game_manage_.getRepeatedCount(count);
}

bool Controller::getGameSpeedLevel(int &level)
{
    return app_game_manage_.getSpeedLevel(level);
}

bool Controller::createManualMovement()
{
    AppGameParamsSub params;
    params.force_level = FORCE_LEVEL_SMALL;
    params.rigidity_level = RIGIDITY_SMALL;
    params.speed_level = SPEED_LEVEL_FAST;
    params.repeated_count = 3;

    if(!createGame(MANUAL_MOVEMENT, params))
    {
        printf("Controller: create game failed.\n");
        return false;  
    }

    if(!startGame())
    {
        printf("Controller: start game failed.\n");
        return false;  
    }
    return true;
}

void Controller::dropManualMovement()
{
    dropGame();
}

bool Controller::createTrajRecurrent()
{
    AppGameParamsSub params;
    params.force_level = FORCE_LEVEL_SMALL;
    params.rigidity_level = RIGIDITY_SMALL;
    params.speed_level = SPEED_LEVEL_FAST;
    params.repeated_count = 3;

    if(!createGame(TRAJ_RECURRENT, params))
    {
        printf("Controller: createGame failed.\n");
        return false;  
    }
    return true;
}

bool Controller::startTrajRecurrent()
{
    if(!startGame())
    {
        printf("Controller: start game failed.\n");
        return false;  
    }
    return true;
}

bool Controller::stopTrajRecurrent()
{
    stopGame();
    return true;
}

bool Controller::dropTrajRecurrent()
{
    dropGame();
    return true;
}

void Controller::manualMoveStateMachine(bool sensor_status)
{

    int game_mode = app_game_manage_.getRunningGameMode();

    if (app_game_manage_.isGameRunning()
        && game_mode == app_game_model::MANUAL_MOVEMENT)
    {
        if (sensor_status)
        {
            usleep(100000);
            return;
        }
        
        usleep(100000);
        dropManualMovement();
        setLampBeltGreen();
        return;
    }

    if (sensor_status && game_mode == 0)
    {
        createManualMovement();
        setLampBeltBlue();
    }
}
