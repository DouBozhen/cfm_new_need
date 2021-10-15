#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <sys/time.h>

#include "app_game_manage.h"
#include "control_arm_rt.h"
#include "control_dashboard.h"
#include "ft_sensor.h"
// #include "control_gpio.h"
#include "training_app_game_manage.h"
#include "training_manual_move.h"
#include "training_traj_recurr.h"

#include "thread_base.h"
#include "thread_pool.h"
#include "app_game_model.h"

#define THREAD_PRIORITY 50

#define INPUT_INDEX_LAMB_BELT_GREEN 0
#define INPUT_INDEX_LAMB_BELT_BLUE 1
#define OUTPUT_INDEX_SENSOR_STATUE 4
#define ROBOT_COMM_COUNT 10

#define JOINT_DELTA_MIN 0.02
#define RETURN_HOME_SPEED_MAX (1.0)
#define RETURN_HOME_WAIT_LOOP_NUM (50)

class Controller: public ThreadBase<Controller>
{
public:
    static Controller* getInstance();
	~Controller();
	bool init();

	bool start();
	void stop();
	void run();
    bool isRunning();

    /* control game */
    bool createGame(int game_mode, app_game_model::AppGameParamsSub sub_params);
    bool startGame();
    void stopGame();
    void dropGame();

    bool setGameForceLevel(int level);
    bool getGameForceLevel(int &level);
	bool setGameRigidityLevel(int level);
	bool setGameRepeatedCount(int count);
	bool setGameSpeedLevel(int level);
	bool getGameRepeatedCount(int &count);
	bool getGameSpeedLevel(int &level);

    bool createTrajRecurrent();
    bool startTrajRecurrent();
    bool stopTrajRecurrent();
    bool dropTrajRecurrent();

    /* control robot */
    void shutDown();
    void powerOff();

    /* for training */
    bool updateHomePosition();
    bool returnHomePosition();
    bool generateTrajProximal();
    bool generateTrajDistal();
    bool updateEndPositionProximal();
    bool updateEndPositionDistal();
    bool startTrainingTrajRercurr();
    bool endTrainingTrajRecurr();

    bool isReachedHomePosition();
    bool isGenerateProximalTrajComplete();
    bool isReturnHomePosition();
    bool isGenerateDismalTrajComplete();

private:
    enum TrainingMoveState{IDLE, TO_MOVE, MOVING, TO_STOP, STOPING, STOPED};
    static Controller* instance_;
    AppGameManage app_game_manage_;
    ControlArmRealtime* control_arm_rt_;
    FTSensor ft_sensor_;
    ControlDashboard* control_db_;

    std::string ft_sensor_ip_;
    std::string robot_arm_ip_;

    app_game_model::TrainingManualMoveType training_manual_move_type_;
    app_game_model::TrainingManualMoveType training_manual_move_type_last_;
    TrainingManualMove* training_manual_move_ ;
    TrainingTrajRecurr training_traj_recurr_;
    struct timeval start_move_time_;
    TrainingMoveState training_move_state_;
    bool is_reached_position_;

	ur_data_type::Joint proximal_limit_position_;
	ur_data_type::Joint distal_limit_position_;
	ur_data_type::Joint home_position_;
    ur_data_type::Joint actual_position_;


    bool startRobot();
    void stopRobot();

    bool createManualMovement();
    void dropManualMovement();

    /* control gpio */
    bool getGpioSensorStatus();
    bool setLampBeltGreen();
    bool setLampBeltBlue();

    /* state machine */
    void manualMoveStateMachine(bool sensor_status);
    void trainingMoveStateMachineUpdateHome(bool sensor_status);
    void trainingMoveStateMachine(bool sensor_status);
    void trainingReturnHome();

    bool createTrainingManualMove();
    void dropTrainingManualMove();

    bool createTrainingTrajRecurr();
    void dropTrainingTrajRecurr();

    Controller();
};

#endif

