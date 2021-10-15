#include <cstring>
#include <unistd.h> 
#include <mutex>

#include "read_file.h"
#include "controller.h"
#include "path_file.h"
#include "thread_pool.h"
#include "app_game_model.h"
#include "force_datatype.h"

using namespace force_data_type;
using namespace app_game_model;

Controller* Controller::instance_ = nullptr;

Controller::Controller()
{
    is_running_ = false;
    control_arm_rt_ = nullptr;
    control_db_  = nullptr;
    app_game_manage_ = AppGameManage();
    ft_sensor_ = FTSensor();
    training_move_state_ = IDLE;

    is_reached_position_ = false;

    training_manual_move_ = nullptr;
}

Controller* Controller::getInstance()
{
    if (instance_ == nullptr)
	{
		instance_ = new Controller();
	}

	return instance_;
}

Controller::~Controller()
{
    //std::mutex lock_;
    //std::unique_lock<std::mutex> l{ lock_ };

    stop();
    printf("to delete controller)_arm_rt_\n");
    if (control_arm_rt_ != nullptr)
    {
        delete control_arm_rt_;
        control_arm_rt_ = nullptr;
    }

    printf("to delete control_db_\n");
    if (control_db_ != nullptr)
    {
        delete control_db_;
        control_db_ = nullptr;
    }

    printf("to training_manual_move_\n");
    if (training_manual_move_ != nullptr)
    {
        dropTrainingManualMove();
    }

    printf("to instance_\n");
    if (instance_ != nullptr)
    {
        delete instance_;
        instance_ = nullptr;
    }
}

bool Controller::init()
{
    if(!readFile(CONFIG_FILE_DIR, ft_sensor_ip_, robot_arm_ip_))
    {
        return false;
    }

    printf("Controller: FT Sensor ip: %s\n", ft_sensor_ip_.c_str());
    printf("Controller: Robot ip: %s\n", robot_arm_ip_.c_str());
    control_arm_rt_ = new ControlArmRealtime(robot_arm_ip_);
    control_db_ = new ControlDashboard(robot_arm_ip_);
    return true;
}

bool Controller::start()
{
    if (is_running_) return true;

    /* start ft sensor*/
	if (!ft_sensor_.start(ft_sensor_ip_))
    {
        printf("Controller: start ft sensor failed.\n");
        return false;
    }
    usleep(2000000);

    for (int i = 0; i != 10; ++i)
    {
        FTSensorData sensor_data = ft_sensor_.getSensorData();

        if (fabs(sensor_data.force.fx) < 0.0001
            && fabs(sensor_data.force.fy) < 0.0001
            && fabs(sensor_data.force.fz) < 0.0001
            && fabs(sensor_data.torque.tx) < 0.0001
            && fabs(sensor_data.torque.ty) < 0.0001
            && fabs(sensor_data.torque.tz) < 0.0001)
        {
            printf("Controller: start ft sensor failed: data error\n");
            sensor_data.printValue("sensor_data: ");
            usleep(1000000);
            continue;
        }

        break;
    }

    if (!startRobot())
    {
        printf("Controller: start robot failed.\n");
        return false;
    }
    usleep(100000);

    training_traj_recurr_.setComponent(control_arm_rt_);
    ThreadPool::getInstance().startAllocatePool();

    training_manual_move_type_ = TRAINING_NONE;
    is_running_ = true;
    ThreadBase<Controller>::start(THREAD_PRIORITY);

    return true;
}

void Controller::stop()
{
    if (!is_running_) 
    {
        return;
    }

    is_running_ = false;
    ThreadBase<Controller>::stop();

    dropGame();
    
    control_db_->halt();
    control_arm_rt_->halt();
    ft_sensor_.stop();
    
    ThreadPool::getInstance().stopAllocatePool();
}

void Controller::run()
{
	bool sensor_status = false;
    setLampBeltGreen();
    while (is_running_)
    {
		sensor_status = getGpioSensorStatus();
        // sensor_status = true;
#if 1
        if (training_manual_move_type_ == UPDATE_TRAINING_HOME_POSITION)
        {
            trainingMoveStateMachineUpdateHome(sensor_status);
            usleep(100000);
            continue;
        }

        if (training_manual_move_type_ == TRAINING_GRNERATE_TRAJ_PROXIMAL
            || training_manual_move_type_ == TRAINING_GRNERATE_TRAJ_DISTAL
            || training_manual_move_type_ == TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL
            || training_manual_move_type_ == TRAINING_UPDATE_LIMIT_POSITION_DISTAL)
        {
            trainingMoveStateMachine(sensor_status);
            usleep(100000);
            continue;
        }

        if (training_manual_move_type_ == TRAINING_RETURN_HOME_POSITION)
        {
            trainingReturnHome();
            usleep(10000);
            continue;
        }
#endif
        //manualMoveStateMachine(sensor_status);
        usleep(100000);
    }
}

bool Controller::isRunning()
{
    return is_running_;
}

