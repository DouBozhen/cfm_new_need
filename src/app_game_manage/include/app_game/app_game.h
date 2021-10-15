#ifndef APP_GAME_H
#define APP_GAME_H

#include <string>

#include "handler.h"
#include "app_game_model.h"
#include "control_arm_rt.h"
#include "force_datatype.h"
#include "ft_sensor.h"

#define THREAD_INDEX 0

class AppGame : public Handler
{
public:
	virtual bool initParams(app_game_model::AppGameParams params) = 0;
	virtual bool start(ControlArmRealtime* control_arm_rt, FTSensor* ft_sensor) = 0;
	virtual void pause() = 0;
	virtual void stop() = 0;
	virtual void replay() = 0;
    virtual bool isRunning() = 0;

    virtual void encodeRuntimeInfo(char* buf, int &buf_size) = 0;
    virtual bool decodeRuntimeCmd(char* buf, int buf_size) = 0;

protected:
    unsigned int training_mode_;
    unsigned int running_state_;
    app_game_model::AppGameParams params_;
    bool remove_zero_bias_;
    std::string path_file_;
    bool is_processing_;

    void generateCmdJointSpeed(ur_data_type::Joint joint_vel,
        ur_data_type::Joint speed, force_data_type::FTSensorData vw,
        bool &remove_zero_bias, ur_data_type::Joint &cmd_speed, double &acc);
};

#endif
