#include <stdio.h>
#include <unistd.h> 
#include "ft_sensor.h"
#include "force_datatype.h"

using namespace force_data_type;

void testCalculate()
{
    ForceValue f1;
    f1.fx = 1.0;
    f1.fy = 1.0;
    f1.fz = 1.0;

    ForceValue f2 = f1 * 0.5;
    printf("%lf, %lf\n", f1.fx, f1.fy);
}




int main(int argc, char** argv)
{
    // testCalculate();
#if 1
    FTSensor ft_sensor = FTSensor();

    if (!ft_sensor.start("192.168.1.1"))
    {
        printf("ft sensor start failed\n");
        return -1;
    }

    FTSensorData sensor_data = ft_sensor.getSensorData();
    usleep(1000000);
    ft_sensor.removeBias();

    int count = 1000;
    int i = 0;
    while (++i != count)
    {
        sensor_data = ft_sensor.getSensorData();
        printf("%d: %f, %f, %f, %f, %f, %f\n", i,
            sensor_data.force.fx, 
            sensor_data.force.fy, 
            sensor_data.force.fz, 
            sensor_data.torque.tx, 
            sensor_data.torque.ty,
            sensor_data.torque.tz
            );
        usleep(100000);
    }

    ft_sensor.stop();
#endif
    return 0;
}


