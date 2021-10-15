#include "controller.h"
#include "rpc_connection.h"

bool RpcConnection::processPoweroff()
{
    Controller::getInstance()->powerOff();
	return true;
}

bool RpcConnection::processStartRobot()
{
	return true; /* Controller::getInstance()->start(); */
}

bool RpcConnection::processCloseRobot()
{
	Controller::getInstance()->stop();
	return true;
}

bool RpcConnection::processTractionForce_SetLevel(int level)
{
	return Controller::getInstance()->setGameForceLevel(level);
}

bool RpcConnection::processTractionForce_GetLevel(int &level)
{
	return Controller::getInstance()->getGameForceLevel(level);
}

bool RpcConnection::processTrajectoryReconstruction_Open()
{
	return Controller::getInstance()->createTrajRecurrent();
}

bool RpcConnection::processTrajectoryReconstruction_Close()
{
	return Controller::getInstance()->dropTrajRecurrent();
}

bool RpcConnection::processTrajectoryReconstruction_SetInfo(int repeat_count, int speed_ratio)
{
	Controller::getInstance()->setGameRepeatedCount(repeat_count);
	Controller::getInstance()->setGameSpeedLevel(speed_ratio);
	return true;
}

bool RpcConnection::processTrajectoryReconstruction_GetInfo(int &repeat_count, int &speed_ratio)
{
	Controller::getInstance()->getGameRepeatedCount(repeat_count);
	Controller::getInstance()->getGameRepeatedCount(speed_ratio);
	return true;
}

bool RpcConnection::processTrajectoryReconstruction_Start()
{
	return Controller::getInstance()->startTrajRecurrent();
}

bool RpcConnection::processTrajectoryReconstruction_End()
{
	return Controller::getInstance()->stopTrajRecurrent();
}

bool RpcConnection::processHeartBeat()
{
	return true;
}

bool RpcConnection::processTrajectoryReconstruction_SetSpeedRatio(int ratio)
{
	return Controller::getInstance()->setGameSpeedLevel(ratio);
}

bool RpcConnection::processTrajectoryReconstruction_GetSpeedRatio(int &ratio)
{
	return Controller::getInstance()->getGameSpeedLevel(ratio);
}

bool RpcConnection::processTrajectoryReconstruction_SetRepeatCount(int count)
{
	return Controller::getInstance()->setGameRepeatedCount(count);
}

bool RpcConnection::processTrajectoryReconstruction_GetRepeatCount(int &count)
{
	return Controller::getInstance()->getGameRepeatedCount(count);
}

bool RpcConnection::processGetSensorStatus()
{
	return false;
}

bool RpcConnection::processImpedanceMotion_Open()
{
	return false;
}

bool RpcConnection::processImpedanceMotion_Close()
{
	return false;
}

bool RpcConnection::processImpedanceMotion_SetRangelevel()
{
	return false;
}

bool RpcConnection::processImpedanceMotion_GetRangelevel()
{
	return false;
}

bool RpcConnection::processGetJointPosition()
{
	return false;
}

bool RpcConnection::processGetCartPosition()
{
	return false;
}

bool RpcConnection::processSetCartPosition()
{
	return false;
}

bool RpcConnection::processRemotePulling_Open()
{
	return false;
}

bool RpcConnection::processRemotePulling_Close()
{
	return false;
}

bool RpcConnection::processOmegaPulling_Open()
{
	return false;
}

bool RpcConnection::processOmegaPulling_Close()
{
	return false;
}

bool RpcConnection::processTraining_UpdateStartPose()
{
	return Controller::getInstance()->updateHomePosition();
}
bool RpcConnection::processTraining_ReturnStartPose()
{
	return Controller::getInstance()->returnHomePosition();
}
bool RpcConnection::processTraining_GenerateTrajProximal()
{
	return Controller::getInstance()->generateTrajProximal();
}
bool RpcConnection::processTraining_GenerateTrajDistal()
{
	return Controller::getInstance()->generateTrajDistal();
}
bool RpcConnection::processTraining_UpdateEndPoseProximal()
{
	return Controller::getInstance()->updateEndPositionProximal();
}
bool RpcConnection::processTraining_UpdateEndPoseDistal()
{
	return Controller::getInstance()->updateEndPositionDistal();
}
bool RpcConnection::processTraining_StartTrajRecurr()
{
	return Controller::getInstance()->startTrainingTrajRercurr();
}
bool RpcConnection::processTraining_EndTrajRecurr()
{
	return Controller::getInstance()->endTrainingTrajRecurr();;
}

bool RpcConnection::processTraining_IsReachedStartPose(bool status)
{
	status = Controller::getInstance()->isReachedHomePosition();
	return status;
}

bool RpcConnection::processTraining_IsReturnStartPose(bool status)
{
	status = Controller::getInstance()->isReturnHomePosition();
	return status;
}

bool RpcConnection::processTraining_IsGenerateProximalTrajComplete(bool status)
{
	status = Controller::getInstance()->isGenerateProximalTrajComplete();
	return status;
}

bool RpcConnection::processTraining_IsGenerateDistalTrajComplete(bool status)
{
	status = Controller::getInstance()->isGenerateDismalTrajComplete();
	return status;
}
