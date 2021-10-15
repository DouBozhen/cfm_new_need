#ifndef APP_GAME_MANAGE_H
#define APP_GAME_MANAGE_H

#include "app_game.h"
#include "manual_movement.h"
#include "puzzle_game.h"
#include "traj_recurrent.h"

class AppGameManage
{
public:
    AppGameManage();
	bool createGame(int game_mode, app_game_model::AppGameParamsSub sub_params);
    void dropGame();
	bool isGameRunning();
	int getRunningGameMode();

	bool startGame(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor);
	void pauseGame();
	void replayGame();
	void stopGame();

	bool setForceLevel(int level);
	bool getForceLevel(int &level);
	bool setRigidityLevel(int level);
	bool setRepeatedCount(int count);
	bool setSpeedLevel(int level);
	bool getRepeatedCount(int &count);
	bool getSpeedLevel(int &level);

private:
    int game_mode_;
	app_game_model::AppGameParams params_;
    AppGame* game_;
};

#endif 
