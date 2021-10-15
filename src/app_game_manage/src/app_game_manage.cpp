#include <stdio.h>
#include <unistd.h> 
#include "app_game_model.h"
#include "app_game_manage.h"

using namespace app_game_model;

AppGameManage::AppGameManage()
{
    game_mode_ = 0;
    game_ = nullptr;
}

bool AppGameManage::createGame(int game_mode, app_game_model::AppGameParamsSub sub_params)
{
    if (game_ != nullptr)
    {
        printf("AppGameManage: game %s is running.\n", game_->getName().c_str());
        return true;
    }

    params_.force_level = FORCE_LEVEL_ZERO; //sub_params.force_level;
    params_.rigidity_level = RIGIDITY_SMALL; //sub_params.rigidity_level;
    params_.repeated_count = 1; //sub_params.repeated_count;
    params_.speed_level = SPEED_LEVEL_MEDIUM; // sub_params.speed_level;

    params_.training_mode = ACTIVE;
    params_.apply_force_mode = Compliance;

    switch (game_mode)
    {
        case MANUAL_MOVEMENT:
        {
            game_ = new ManualMovement();
            params_.need_record = true;
            params_.sensor_info.is_sensor_used = true;
            params_.sensor_info.remove_zero_bias = true;
            params_.sensor_info.sensor_ip = sub_params.sensor_ip;
        }
        break;
        case TRAJ_RECURRENT:
        {
            game_ = new TrajRecurrent();
            params_.need_record = false;
            params_.sensor_info.is_sensor_used = false;
        }
        break;
        case PUZZLE_GAME:
        {
            game_ = new PuzzleGame();
            params_.need_record = false;
            params_.sensor_info.is_sensor_used = true;
            params_.sensor_info.remove_zero_bias = true;
            params_.sensor_info.sensor_ip = sub_params.sensor_ip;
        }
        break;
        default:;
    };

    if (game_ == nullptr)
    {
        printf("AppGameManage: create game failed: game mode error\n");
        return false;
    }

    game_mode_ = game_mode;
    return true;
}

void AppGameManage::dropGame()
{
    if (game_ != nullptr)
    {
        if (game_->isRunning())
        {
            stopGame();
            usleep(100000);
        }

        delete game_;
        game_ = nullptr;
    }

    game_mode_ = 0;
}

int AppGameManage::getRunningGameMode()
{
    return game_mode_;
}

bool AppGameManage::startGame(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor)
{
    if (game_ == nullptr)
    {
        printf("AppGameManage: Robot is not started.\n");
        return false;
    }

    if (game_->isRunning())
    {
        printf("AppGameManage: Game is running.\n");
        return true;
    }

    if (!game_->initParams(params_))
    {
        printf("AppGameManage: create game failed: init game error\n");
        return false;
    }

    if (!game_->start(control_arm_rt, ft_sensor))
    {
        printf("AppGameManage: start game failed\n");
        return false;
    }

    return true;
}

void AppGameManage::pauseGame()
{
    if (game_ == nullptr || game_mode_ == 0)
    {
        printf("AppGameManage: game is not started.\n");
        return;
    }

    game_->pause();
}

void AppGameManage::stopGame()
{
    if (game_ == nullptr || game_mode_ == 0)
    {
        //printf("AppGameManage: game is not started.\n");
        return;
    }

    game_->stop();
    game_mode_ = 0;
}

void AppGameManage::replayGame()
{
    if (game_ == nullptr || game_mode_ == 0)
    {
        printf("AppGameManage: game is not started.\n");
        return;
    }

    game_->replay();
}

bool AppGameManage::setForceLevel(int level)
{
    params_.force_level = level;
    return true;
}

bool AppGameManage::setRigidityLevel(int level)
{
    params_.rigidity_level = level;
    return true;
}

bool AppGameManage::setRepeatedCount(int count)
{
    params_.repeated_count = count;
    return true;
}

bool AppGameManage::setSpeedLevel(int level)
{
    params_.speed_level = level;
    return true;
}

bool AppGameManage::isGameRunning()
{
    if (game_ != nullptr)
        return game_->isRunning();
        
    return false;
}

bool AppGameManage::getForceLevel(int &level)
{
     level = params_.force_level;
     return true;
}

bool AppGameManage::getRepeatedCount(int &count)
{
    count = params_.repeated_count;
    return true;
}

bool AppGameManage::getSpeedLevel(int &level)
{
    level = params_.speed_level;
    return true;
}
