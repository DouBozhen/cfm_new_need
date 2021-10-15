#include "rpc_connection.h"
using namespace ForceMasterProto;

void RpcConnection::initRpcTable()
{
    RpcService rpc_service;
    rpc_service = { Poweroff,	&RpcConnection::handlePoweroff };	 rpc_table_.push_back(rpc_service);
    rpc_service = { StartRobot, &RpcConnection::handleStartRobot };	 rpc_table_.push_back(rpc_service);
	rpc_service = { CloseRobot,	&RpcConnection::handleCloseRobot };	 rpc_table_.push_back(rpc_service);

	rpc_service = { TractionForce_SetLevel,	&RpcConnection::handleTractionForce_SetLevel };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TractionForce_GetLevel,	&RpcConnection::handleTractionForce_GetLevel };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_Open,	&RpcConnection::handleTrajectoryReconstruction_Open };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_Close, &RpcConnection::handleTrajectoryReconstruction_Close };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_SetInfo,	&RpcConnection::handleTrajectoryReconstruction_SetInfo };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_GetInfo,	&RpcConnection::handleTrajectoryReconstruction_GetInfo };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_Start,	&RpcConnection::handleTrajectoryReconstruction_Start };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_End,	&RpcConnection::handleTrajectoryReconstruction_End };	 rpc_table_.push_back(rpc_service);

	rpc_service = { HeartBeat,	&RpcConnection::handleHeartBeat };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_SetSpeedRatio,	&RpcConnection::handleTrajectoryReconstruction_SetSpeedRatio };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_GetSpeedRatio,	&RpcConnection::handleTrajectoryReconstruction_GetSpeedRatio };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_SetRepeatCount,	&RpcConnection::handleTrajectoryReconstruction_SetRepeatCount };	 rpc_table_.push_back(rpc_service);
	rpc_service = { TrajectoryReconstruction_GetRepeatCount,	&RpcConnection::handleTrajectoryReconstruction_GetRepeatCount };	 rpc_table_.push_back(rpc_service);

	rpc_service = { Training_UpdateStartPose,	    	&RpcConnection::handleTraining_UpdateStartPose };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_ReturnStartPose,	    	&RpcConnection::handleTraining_ReturnStartPose };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_GenerateTrajProximal,		&RpcConnection::handleTraining_GenerateTrajProximal };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_GenerateTrajDistal,		&RpcConnection::handleTraining_GenerateTrajDistal };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_UpdateEndPoseProximal,		&RpcConnection::handleTraining_UpdateEndPoseProximal };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_UpdateEndPoseDistal,		&RpcConnection::handleTraining_UpdateEndPoseDistal };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_StartTrajRecurr,	    	&RpcConnection::handleTraining_StartTrajRecurr };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_EndTrajRecurr,	        	&RpcConnection::handleTraining_EndTrajRecurr };	 rpc_table_.push_back(rpc_service);

	rpc_service = { Training_IsReachedStartPose,	        	&RpcConnection::handleTraining_IsReachedStartPose };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_IsReturnStartPose,	        	&RpcConnection::handleTraining_IsReturnStartPose };	 rpc_table_.push_back(rpc_service);

	rpc_service = { Training_IsGenerateProximalTrajComplete, &RpcConnection::handleTraining_IsGenerateProximalTrajComplete };	 rpc_table_.push_back(rpc_service);
	rpc_service = { Training_IsGenerateDistalTrajComplete, &RpcConnection::handleTraining_IsGenerateDistalTrajComplete };	 rpc_table_.push_back(rpc_service);

	/* useless */
	rpc_service = { GetSensorStatus,	    &RpcConnection::handleGetSensorStatus };	 rpc_table_.push_back(rpc_service);
	rpc_service = { ImpedanceMotion_Open,   &RpcConnection::handleImpedanceMotion_Open };	 rpc_table_.push_back(rpc_service);
	rpc_service = { ImpedanceMotion_Close,	&RpcConnection::handleImpedanceMotion_Close };	 rpc_table_.push_back(rpc_service);
	rpc_service = { ImpedanceMotion_SetRangelevel,	&RpcConnection::handleImpedanceMotion_SetRangelevel };	 rpc_table_.push_back(rpc_service);
	rpc_service = { ImpedanceMotion_GetRangelevel,	&RpcConnection::handleImpedanceMotion_GetRangelevel };	 rpc_table_.push_back(rpc_service);
	rpc_service = { GetJointPosition,   &RpcConnection::handleGetJointPosition };	 rpc_table_.push_back(rpc_service);
	rpc_service = { GetCartPosition,	&RpcConnection::handleGetCartPosition };	 rpc_table_.push_back(rpc_service);
	rpc_service = { SetCartPosition,	&RpcConnection::handleSetCartPosition };	 rpc_table_.push_back(rpc_service);

	rpc_service = { RemotePulling_Open,	&RpcConnection::handleRemotePulling_Open };	 rpc_table_.push_back(rpc_service);
	rpc_service = { RemotePulling_Close,&RpcConnection::handleRemotePulling_Close };	 rpc_table_.push_back(rpc_service);
	rpc_service = { OmegaPulling_Open,	&RpcConnection::handleOmegaPulling_Open };	 rpc_table_.push_back(rpc_service);
	rpc_service = { OmegaPulling_Close,	&RpcConnection::handleOmegaPulling_Close };	 rpc_table_.push_back(rpc_service);
}