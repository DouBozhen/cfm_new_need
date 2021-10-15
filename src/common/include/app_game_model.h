#ifndef APP_GAME_MODEL_H
#define APP_GAME_MODEL_H

#include <string>

namespace app_game_model
{
    enum GameMode
    {
        MANUAL_MOVEMENT = 1,
        TRAJ_RECURRENT =2,
        PUZZLE_GAME = 3,
        TRAINING_MANUAL_MOVE = 4,
        TRAINING_TRAJ_RECURRENT = 5,
    };

    enum TrainingManualMoveType
    {
        TRAINING_NONE = 0,
        UPDATE_TRAINING_HOME_POSITION = 1,
        TRAINING_GRNERATE_TRAJ_PROXIMAL = 2,
        TRAINING_GRNERATE_TRAJ_DISTAL = 3,
        TRAINING_UPDATE_LIMIT_POSITION_PROXIMAL = 4,
        TRAINING_UPDATE_LIMIT_POSITION_DISTAL = 5,
        TRAINING_RETURN_HOME_POSITION = 6,
    };

    enum TrainingMode
	{
		NONE = 0,
		ASSIST = 1,
		ACTIVE = 2,
        IMPEDANCE = 3,
        PASSIVE = 4
	};
	
    enum GameForceLevel
	{
        FORCE_LEVEL_ZERO = 0,
        FORCE_LEVEL_SMALL = 1,
        FORCE_LEVEL_MEDIUM = 2,
        FORCE_LEVEL_LARGE = 3
	};

	enum GameRunningState
	{
		IDEL = 0,
		PROCESSING = 1,//开始
		WAITING =2,//暂停
		STOPING = 3,//停止
		REPLAYING = 4,//重玩
	};

	enum GameRigidity
	{
        RIGIDITY_ZERO = 0,
        RIGIDITY_SMALL = 1,
        RIGIDITY_MEDIUM = 2,
        RIGIDITY_LARGE = 3
	};
	
    enum GamePathWidth
	{
		PathWidthZero = 0,
		PathWidthSmall = 3,
		PathWidthMedium = 2,
		PathWidthLarge = 1,
	};

	enum GameSpeedLevel
	{
        SPEED_LEVEL_ZERO = 0,
        SPEED_LEVEL_SLOW = 1,
        SPEED_LEVEL_MEDIUM = 2,
        SPEED_LEVEL_FAST = 3
	};

    enum ApplyForceMode
    {
        Unknow = 0,
        Compliance = 1, // 顺从力，顺从末端施加的力，主动力
        Buoyancy = 2, // 浮力
        Random = 3, // 随机力
        Resistance = 4, // 抵抗力
        Plane = 5, // 平面运动力
        Constant = 6, // 恒力
        Passive=7, // 被动力
    };

     struct SensorInfo
    {
        bool is_sensor_used;
        bool remove_zero_bias;
        std::string sensor_ip;
    };

    struct AppGameParamsSub
    {
        unsigned int force_level;
        unsigned int rigidity_level;
        int repeated_count;
        unsigned int speed_level;
        std::string sensor_ip;
        int training_move_type;
        bool need_record;
    };

    struct AppGameParams
    {
        unsigned int force_level;
        unsigned int rigidity_level;
        int repeated_count;
        unsigned int speed_level;
        int training_move_type;

        TrainingMode training_mode;
        ApplyForceMode apply_force_mode;
        bool need_record;
        SensorInfo sensor_info;
    };

    enum PuzzleGameAction{
        PUZZEL_IDEL = 0,
        MOVING = 1,
        CUTTING = 2,
        APARTING = 3,
        FIXTING = 4,
    };

    enum PuzzleFixingState{
        ENTER_FIXING = 0,
        TO_TARGET = 1,
        TO_CENTER = 2,
        EXIT_FIXING = 3,
    };

    enum GameOrder
    {
        ORDINAL = 0,
        INVERSE = 1,
    };
}

#endif