
#ifndef FT_SENSOR_H
#define FT_SENSOR_H

#include <vector>
#include <array>
#include <math.h>
#include <stdio.h>
#include <pthread.h>

#include "thread_base.h"
#include "udp_client.h"
#include "force_datatype.h"

#define UDP_SPORT 49152
#define REQUEST_DATA_BYTES 8
#define FT_SENSOR_UDP_CONNECT_MAX_COUNT 10
#define FT_SENSOR_UDP_CLIENT_BUF_SIZE 36

#define COMMAND 2
#define NUM_SAMPLES 0

struct Response
{
	uint32_t rdt_sequence;
	uint32_t ft_sequence;
	uint32_t status;
	int32_t ft_data[6];
};

class FTSensor : public ThreadBase<FTSensor>
{
public:
    FTSensor();
    ~FTSensor();

	bool start(std::string ip, bool zero_sensor = true);
	void run();
	void stop();
	bool removeBias();
    force_data_type::FTSensorData getSensorData();

	inline float lowFilter(float input, float low_pass_filter_value)
	{
		float b = 2 * float(M_PI) * 3 * 0.002;
		float a = b / (1 + b);
		low_pass_filter_value=a * input + (1 - a)*low_pass_filter_value;//input:本次采样值 getState():上次滤波值
		return low_pass_filter_value;
	}

private:
    UdpClient* udp_client_;

	pthread_mutex_t mutex_;
    force_data_type::FTSensorData sensor_data_;

    uint8_t request_[REQUEST_DATA_BYTES];
	uint8_t request_filter_[REQUEST_DATA_BYTES];       /* The filter request data sent to the optoforce. */
	uint8_t request_output_rate_[REQUEST_DATA_BYTES];  /* The output rate request data sent to the optoforce. */
	uint8_t request_bias_[REQUEST_DATA_BYTES];

	force_data_type::FTSensorData force_sum_;
	force_data_type::FTSensorData force_avg_;
	force_data_type::FTSensorData force_value_;
	std::vector<force_data_type::FTSensorData> smooth_res_;

    uint32_t queue_count_;
	uint32_t filter_wid_;

	bool is_remove_bias_;

    bool is_processing_;
	int connect_times_;
	bool connected_;
	void decodeSensorData(uint8_t* buf, double scale_factor);
	//void initRemoveBiasParams();
};

#endif
