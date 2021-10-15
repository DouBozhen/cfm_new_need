#include <unistd.h>

#include "path_file.h"
#include "app_game_manage.h"
#include "app_game_model.h"
#include "generate_path.h"
#include "training_manual_move.h"

using namespace std;
using namespace app_game_model;

bool testGeneratePath()
{
    GeneratePath generate_path = GeneratePath();
    generate_path.generateJointPathFromFile(JOINT_PATH_FILE_DIR);

    ur_data_type::Joint start_joint = generate_path.getJointStartPosition();
    std::vector<ur_data_type::Joint> joint_path = generate_path.getJointPath();
    if (joint_path.size() < 1)
    {
        printf("joint path is %d\n", joint_path.size());
        return false;
    }

    printf("joint path: \n");
    for (int i = 0; i != joint_path.size(); ++i)
    {
        printf("%d: %lf, %lf, %lf, %lf, %lf, %lf\n", i, 
            joint_path[i].jVal[0], joint_path[i].jVal[1], joint_path[i].jVal[2], 
            joint_path[i].jVal[3], joint_path[i].jVal[4], joint_path[i].jVal[5]);
    }

    return true;
}


void testGameProcess()
{
    AppGameParams params;
    params.training_mode = ASSIST;
    params.force_level = FORCE_LEVEL_SMALL;
    params.rigidity_level = RIGIDITY_SMALL;
    params.apply_force_mode = Compliance;
    params.speed_level = SPEED_LEVEL_SLOW;
    params.repeated_count = 3;

    params.sensor_info.is_sensor_used = false;
    params.need_record = false;
    TrajRecurrent game = TrajRecurrent();
    game.initParams(params);
    game.start(nullptr, nullptr);
	// printf("------------------------ 1st ------------------\n");
    // game.process();
	// printf("------------------------ 2st ------------------\n");
    // game.process();
	// printf("------------------------ 3st ------------------\n");
    // game.process();
//    game.stop();
}

#if 0
void testTraningManualMove()
{
    AppGameParams params;
    params.force_level = FORCE_LEVEL_MEDIUM;
    params.rigidity_level = RIGIDITY_SMALL;
    params.repeated_count = 1;
    params.speed_level = SPEED_LEVEL_SLOW;
    params.training_move_type = UPDATE_TRAINING_HOME_POSITION;
    params.training_mode = PASSIVE;    
    params.apply_force_mode = Compliance;
    params.need_record = false;
    params.sensor_info.is_sensor_used = true;
    params.sensor_info.remove_zero_bias = false;
    params.sensor_info.sensor_ip = "172.31.1.150";

    TrainingManualMove* training_manual_move = new TrainingManualMove();

    if (!training_manual_move->initParams(params))
    {
        printf("Controller: initial Training Manual Move failed.\n");
        return false;
    }

    if (!training_manual_move->start(control_arm_rt_, &ft_sensor_))
    {
        printf("Controller: start Training Manual Move failed.\n");
        return false;
    }

    int count = 60;
    int i = 0;
    while(++i < count)
    {
        usleep(1000000);
    }

    if (training_manual_move->isRunning())
    {
        training_manual_move->stop();
    }

    delete training_manual_move;
    training_manual_move = nullptr;
}
#endif

int main()
{
#if 0
    AppGameParams params;
    params.training_mode = ASSIST;
    params.force_level = FORCE_LEVEL_SMALL;
    params.rigidity_level = RIGIDITY_SMALL;
    params.apply_force_mode = Compliance;
    params.speed_level = SPEED_LEVEL_SLOW;
    params.repeated_count = 3;

    params.sensor_info.is_sensor_used = false;
    params.need_record = false;

    AppGameManage app_game_manager;
    if(!app_game_manager.createGame(MANUAL_MOVEMENT, params))
    {
        return -1;
    }

    app_game_manager.dropGame();
#endif
    // testGameProcess();

    //testTraningManualMove();
    //if (!testGeneratePath()) return -1;
    return 0;
}

