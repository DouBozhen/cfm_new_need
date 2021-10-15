#include <unistd.h>
#include <sys/time.h>
#include "test_tcp_comm.h"

using namespace ForceMasterProto;

bool TestTcpComm::trestTrainingUpdateStartPose()
{
    return processCmd(Training_UpdateStartPose);
}

bool TestTcpComm::trestTrainingReturnStartPose()
{
    return processCmd(Training_ReturnStartPose);
}

bool TestTcpComm::trestTrainingGenerateTrajProximal()
{
    return processCmd(Training_GenerateTrajProximal);
}

bool TestTcpComm::trestTrainingGenerateTrajDistal()
{
    return processCmd(Training_GenerateTrajDistal);
}

bool TestTcpComm::trestTrainingUpdateEndPoseProximal()
{
    return processCmd(Training_UpdateEndPoseProximal);
}

bool TestTcpComm::trestTrainingUpdateEndPoseDistal()
{
    return processCmd(Training_UpdateEndPoseDistal);
}

bool TestTcpComm::trestTrainingStartTrajRecurr()
{
    return processCmd(Training_StartTrajRecurr);
}

bool TestTcpComm::trestTrainingEndTrajRecurr()
{
    return processCmd(Training_EndTrajRecurr);
}

bool TestTcpComm::trestTrainingIsReachedHomePose()
{
    return processGetStatusCmd(Training_IsReachedStartPose);
}

bool TestTcpComm::trestTrainingIsReturnHomePose()
{
    return processGetStatusCmd(Training_IsReturnStartPose);
}

bool TestTcpComm::trestTrainingIsGenerateTrajProximalComplete()
{
    return processGetStatusCmd(Training_IsGenerateProximalTrajComplete);
}

bool TestTcpComm::trestTrainingIsGenerateTrajDistalComplete()
{
    return processGetStatusCmd(Training_IsGenerateDistalTrajComplete);
}

