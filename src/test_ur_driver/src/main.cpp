
#include <string>
#include <stdio.h>
#include <iostream>
#include <unistd.h>

//#include "test_class.h"
#include "test_dashboard.h"
#include "test_control_arm.h"
#include "test_control_arm_rt.h"

using namespace std;

void testDB(string ip)
{
    TestControlDashboard testdb(ip);
    testdb.testGetParams();
}

void testControlArmRT(string ip)
{
    TestControlArmRT test_arm_rt(ip);
    // test_arm_rt.testGetParams();
    if(test_arm_rt.testSendScript())
    {
        printf("testSendScript successful\n");
    }
    else
    {
        printf("testSendScript failed\n");
    }
}

void testControlArm(string ip)
{
    TestControlArm test_arm(ip);
    test_arm.testGetParams();
}

int main()
{
    string ip("192.168.1.10");
    printf("Hello World\n");
    // testDB(ip);
     testControlArmRT(ip);
    // testControlArm(ip);
    return 0;
}

