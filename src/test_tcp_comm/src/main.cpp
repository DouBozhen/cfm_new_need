#include <unistd.h>
#include "test_tcp_comm.h"

void testParams()
{
    TestTcpComm test_tcp_comm;
    if (!test_tcp_comm.init())
    {
        printf("init test_tcp_comm failed.\n");
        return;
    }
    usleep(1000000);
    printf("----------- testHeartBeat -----------\n");
    test_tcp_comm.testAllParams();
    // printf("----------- testSetForceLevel -----------\n");
    // test_tcp_comm.testSetForceLevel(1);
    // usleep(2000000);
    // printf("----------- testGetForceLevel -----------\n");
    // test_tcp_comm.testGetForceLevel();
    // printf("----------- testSetRepeatedCount -----------\n");
    // test_tcp_comm.testSetRepeatedCount(3);
    // printf("----------- testGetRepeatedCount -----------\n");
    // test_tcp_comm.testGetRepeatedCount();
    // printf("----------- testSetSpeedRatio -----------\n");
    // test_tcp_comm.testSetSpeedRatio(2);
    // printf("----------- testGetSpeedRatio -----------\n");
    // test_tcp_comm.testGetSpeedRatio();
    usleep(5000000);
}

void testTrajRecurr()
{
    TestTcpComm test_tcp_comm;
    if (!test_tcp_comm.init())
    {
        printf("init test_tcp_comm failed.\n");
        return;
    }

    if (!test_tcp_comm.testOpenTrajRecurr())
    {
        printf("open TrajRecurr failed.\n");
        return;
    }

    if (!test_tcp_comm.testStartTrajRecurr())
    {
        printf("start TrajRecurr failed.\n");
        return;
    }

    usleep(30000000); /* 30s */

    test_tcp_comm.testStopTrajRecurr();
    test_tcp_comm.testCloseTrajRecurr();
}

void testTraining()
{
    TestTcpComm test_tcp_comm;

    if (!test_tcp_comm.init())
    {
        printf("init test_tcp_comm failed.\n");
        return;
    }

    if (!test_tcp_comm.trestTrainingUpdateStartPose())
    {
        printf("start TrajRecurr failed.\n");
        return;
    }

    if (!test_tcp_comm.trestTrainingGenerateTrajProximal())
    {
        printf("start TrajRecurr failed.\n");
        return;
    }

    if (!test_tcp_comm.trestTrainingGenerateTrajDistal())
    {
        printf("start TrajRecurr failed.\n");
        return;
    }

    if (!test_tcp_comm.trestTrainingUpdateEndPoseProximal())
    {
        printf("start TrajRecurr failed.\n");
        return;
    }

    if (!test_tcp_comm.trestTrainingUpdateEndPoseDistal())
    {
        printf("start TrajRecurr failed.\n");
        return;
    }

}

int main()
{
    // testParams();
    //testTrajRecurr();
    testTraining();

    return 0;
}

