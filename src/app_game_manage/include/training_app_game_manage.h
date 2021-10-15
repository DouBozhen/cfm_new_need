#ifndef TRAINING_APP_GAME_MANAGE_H
#define TRAINING_APP_GAME_MANAGE_H

#include "training_manual_move.h"
#include "training_traj_recurr.h"

class TrainingAppGameManage
{
public:
    TrainingAppGameManage(){}

private:
    TrainingManualMove manual_move_;
    TrainingTrajRecurr traj_recurr_;
	app_game_model::AppGameParams params_;
};

#endif 
