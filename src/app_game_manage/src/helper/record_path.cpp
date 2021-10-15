#include <fstream>
#include <cstring>
#include <string>
#include "record_path.h"
#include "thread_pool.h"
#include "path_file.h"
using namespace ur_data_type;
using namespace math_calculate;

#define SET_RECORD_LIMIT_MAX

RecordPath::RecordPath()
{
	joint_file_ = JOINT_PATH_FILE_DIR;
	cart_pose_file_ = CART_PATH_FILE_DIR;
	joint_file_impendance_ = JOINT_IMPENDANCE_PATH_FILE_DIR;
	cart_pose_file_impendance_ = CART_IMPENDANCE_PATH_FILE_DIR;
    cart_path_.clear();
    joint_path_.clear();
    cart_path_impendance_.clear();
    joint_path_impendance_.clear();

    impendance_count_ = 0;

	thread_index_ = 1;
	is_running_ = false;
	sleep_count_ = 0;

    memset(&record_limit_, 0, sizeof(record_limit_));
}

RecordPath::~RecordPath()
{
    cart_path_.clear();
    joint_path_.clear();
    cart_path_impendance_.clear();
    joint_path_impendance_.clear();

    stop();
}

void RecordPath::setRecordLimit(RecordLimit limit)
{
	record_limit_ = limit;
}

bool RecordPath::start(ControlArmRealtime* robot_rt, 
	struct RecordLimit limit,
	std::string cart_pose_file,
	std::string cart_pose_file_impendance,
	std::string joint_file,
	std::string joint_file_impendance)
{
    if(robot_rt == nullptr) 
    {
        printf("RecordPath: robot_rt pointer is nunnptr\n");
        return false;
    }

	joint_file_ = joint_file;
	cart_pose_file_ = cart_pose_file;
	joint_file_impendance_ = joint_file_impendance;
	cart_pose_file_impendance_ = cart_pose_file_impendance;

	joint_path_.clear();
	cart_path_.clear();
	joint_path_impendance_.clear();
	cart_path_impendance_.clear();

    robot_rt_ = robot_rt;
    
	joint_ = robot_rt_->getActualJoint();
    if (joint_.jVal[0] < DOUBLE_ZERO
            && joint_.jVal[1] < DOUBLE_ZERO
            && joint_.jVal[2] < DOUBLE_ZERO
            && joint_.jVal[3] < DOUBLE_ZERO
            && joint_.jVal[4] < DOUBLE_ZERO
            && joint_.jVal[5] < DOUBLE_ZERO
    )
    {
        printf("RecordPath: joint data error\n");
        return false;
    }

	joint_path_.push_back(joint_); // 存入轨迹复现轨迹
	joint_path_impendance_.push_back(joint_); // 存入阻抗复现轨迹
	
	double rotation[3][3];
    ur_kine_.FK(joint_, cart_pose_, rotation);
	cart_path_.push_back(cart_pose_);
	cart_path_impendance_.push_back(cart_pose_);
    
    is_running_ = true;

	ThreadPool::getInstance().registerHandler(this, RECORD_PATH_THREAD_INDEX);
	printf("RecordPath is running!\n");
	return true;
}

void RecordPath::setFiles(
		ControlArmRealtime* robot_rt,
		std::string cart_pose_file,
		std::string cart_pose_file_impendance,
		std::string joint_file,
		std::string joint_file_impendance	
)
{
	robot_rt_ = robot_rt;
	joint_file_ = joint_file;
	cart_pose_file_ = cart_pose_file;
	joint_file_impendance_ = joint_file_impendance;
	cart_pose_file_impendance_ = cart_pose_file_impendance;
}

void RecordPath::process()
{
	if (!is_running_) return;

	if (sleep_count_ < 10)
	{
		sleep_count_++;
		return;
	}
	sleep_count_ = 0;

	joint_ = robot_rt_->getActualJoint();

	if (fabs(joint_.jVal[0]) < 0.01
		&& fabs(joint_.jVal[1]) < 0.01
		&& fabs(joint_.jVal[2]) < 0.01
		&& fabs(joint_.jVal[3]) < 0.01
		&& fabs(joint_.jVal[4]) < 0.01
		&& fabs(joint_.jVal[5]) < 0.01)
	{
		printf("TrackerRecord: joint value error\n");
		return;
	}

	getJointPath(joint_, impendance_count_);

	double rotation[3][3];
    ur_kine_.FK(joint_, cart_pose_, rotation);

	getCartPath(cart_pose_, impendance_count_);

	if (impendance_count_ >= 5)
	{
		impendance_count_ = 0;
	}
	else
	{
		impendance_count_++;
	}
}

void RecordPath::stopRecordEndPositions()
{
	if (!is_running_) return;

	is_running_ = false;
	ThreadPool::getInstance().deregisterHandler(RECORD_PATH_THREAD_INDEX);

	printf("stopRecordEndPositions: path size: %d, %d, %d, %d\n", 
		static_cast<int>(joint_path_.size()), 
		static_cast<int>(joint_path_impendance_.size()), 
		static_cast<int>(cart_path_.size()), 
		static_cast<int>(cart_path_impendance_.size()));

	scaleRpyDistanceP2P(cart_pose_last_.rpy, cart_pose_.rpy);
	if (getP2PDistance(cart_pose_last_.point, cart_pose_.point) > 0.01)
	{
		cart_path_.push_back(cart_pose_);
		cart_path_impendance_.push_back(cart_pose_);
	}

	if (maxJointDistanceP2P(joint_last_, joint_) > 0.02)
	{
		joint_path_.push_back(joint_); // 存入轨迹复现轨迹
		joint_path_impendance_.push_back(joint_); // 存入阻抗复现轨迹
	}

	addEndingPositionsInJointPath(joint_file_);
	addEndingPositionsInJointPathImpendance(joint_file_impendance_);
	addEndingPositionsInCartPath(cart_pose_file_);
	addEndingPositionsInCartPathImpendance(cart_pose_file_impendance_);
}

void RecordPath::stop()
{
	if (!is_running_) return;

	is_running_ = false;
	ThreadPool::getInstance().deregisterHandler(RECORD_PATH_THREAD_INDEX);

	scaleRpyDistanceP2P(cart_pose_last_.rpy, cart_pose_.rpy);
	if (getP2PDistance(cart_pose_last_.point, cart_pose_.point) > 0.01)
	{
		cart_path_.push_back(cart_pose_);
		cart_path_impendance_.push_back(cart_pose_);
	}

	if (maxJointDistanceP2P(joint_last_, joint_) > 0.02)
	{
		joint_path_.push_back(joint_); // 存入轨迹复现轨迹
		joint_path_impendance_.push_back(joint_); // 存入阻抗复现轨迹
	}

	printf("RecordPath: path size: %d, %d, %d, %d\n", 
		static_cast<int>(joint_path_.size()), 
		static_cast<int>(joint_path_impendance_.size()), 
		static_cast<int>(cart_path_.size()), 
		static_cast<int>(cart_path_impendance_.size()));

	if (!joint_path_.empty())
	{
		recordJointPath(joint_file_);
	}
	if (!joint_path_impendance_.empty())
	{
		recordJointPathImpendance(joint_file_impendance_);
	}
	if (!cart_path_.empty())
	{
		recorCartPath(cart_pose_file_);
	}
	if (!cart_path_impendance_.empty())
	{
		recordCartPathImpendance(cart_pose_file_impendance_);
	}
	printf("RecordPath: record path successful. handler removed\n");
}

bool RecordPath::isRunning()
{
    return is_running_;
}

void RecordPath::getJointPath(ur_data_type::Joint joint, int impendance_count)
{
	/* 处理第一个轨迹点 */
	if (joint_path_.empty())
	{
		joint_path_.push_back(joint);
		joint_path_impendance_.push_back(joint);
		joint_last_ = joint;
		return;
	}

	/* 处理余下的点 */
	if (maxJointDistanceP2P(joint_last_, joint) < record_limit_.joint_p2p_distance_min)
	{
		return;
	}

	joint_path_.push_back(joint); // 存入轨迹复现轨迹
	if (impendance_count > 5)
	{
		joint_path_impendance_.push_back(joint); // 存入阻抗复现轨迹
	}

	joint_last_ = joint;
}

bool RecordPath::addEndingPositionsInJointPath(std::string file)
{
	printf("------------------- addEndingPositionsInJointPath %d,-----------------\n", joint_path_.size());
	printf("addEndingPositionsInJointPath: %s\n", file.c_str());
	
	std::ofstream file_joint(joint_file_.c_str(), ios::app);
	for (int i = 0; i != static_cast<int>(joint_path_.size()); ++i)
	{
			printf("joint_path_[%d]: %lf, %lf, %lf, %lf, %lf, %lf\n",
				i, 
				joint_path_[i].jVal[0],
				joint_path_[i].jVal[1],
				joint_path_[i].jVal[2],
				joint_path_[i].jVal[3],
				joint_path_[i].jVal[4],
				joint_path_[i].jVal[5]
				);

		file_joint
			<< joint_path_[i].jVal[0] << std::string(",")
			<< joint_path_[i].jVal[1] << std::string(",")
			<< joint_path_[i].jVal[2] << std::string(",")
			<< joint_path_[i].jVal[3] << std::string(",")
			<< joint_path_[i].jVal[4] << std::string(",")
			<< joint_path_[i].jVal[5] << std::endl;
	}
	file_joint.close();
}

bool RecordPath::addEndingPositionsInJointPathImpendance(std::string file)
{
	std::ofstream file_joint(joint_file_impendance_.c_str(), ios::app);
	for (int i = 0; i != static_cast<int>(joint_path_impendance_.size()); ++i)
	{
		file_joint
			<< joint_path_impendance_[i].jVal[0] << std::string(",")
			<< joint_path_impendance_[i].jVal[1] << ","
			<< joint_path_impendance_[i].jVal[2] << ","
			<< joint_path_impendance_[i].jVal[3] << ","
			<< joint_path_impendance_[i].jVal[4] << ","
			<< joint_path_impendance_[i].jVal[5] << std::endl;
	}
	file_joint.close();
}

bool RecordPath::addEndingPositionsInCartPath(std::string file)
{
	std::ofstream file_joint(cart_pose_file_.c_str(), ios::app);
	for (int i = 0; i != static_cast<int>(cart_path_.size()); ++i)
	{
		file_joint
			<< cart_path_[i].point.x << std::string(",")
			<< cart_path_[i].point.y << ","
			<< cart_path_[i].point.z << ","
			<< cart_path_[i].rpy.rx << ","
			<< cart_path_[i].rpy.ry << ","
			<< cart_path_[i].rpy.rz << std::endl;
	}
	file_joint.close();
}

bool RecordPath::addEndingPositionsInCartPathImpendance(std::string file)
{
	std::ofstream file_joint(cart_pose_file_impendance_.c_str(), ios::app);
	for (int i = 0; i != static_cast<int>(cart_path_impendance_.size()); ++i)
	{
		file_joint
			<< cart_path_impendance_[i].point.x << std::string(",")
			<< cart_path_impendance_[i].point.y << ","
			<< cart_path_impendance_[i].point.z << ","
			<< cart_path_impendance_[i].rpy.rx << ","
			<< cart_path_impendance_[i].rpy.ry << ","
			<< cart_path_impendance_[i].rpy.rz << std::endl;
	}
	file_joint.close();	
}


void RecordPath::recordJointPath(std::string file)
{
	std::ofstream file_joint(joint_file_.c_str(), fstream::out|ios_base::trunc);
	for (int i = 0; i != static_cast<int>(joint_path_.size()); ++i)
	{
		file_joint
			<< joint_path_[i].jVal[0] << std::string(",")
			<< joint_path_[i].jVal[1] << ","
			<< joint_path_[i].jVal[2] << ","
			<< joint_path_[i].jVal[3] << ","
			<< joint_path_[i].jVal[4] << ","
			<< joint_path_[i].jVal[5] << std::endl;
	}
	file_joint.close();
}

void RecordPath::recordJointPathImpendance(std::string file)
{
	std::ofstream file_joint_impendance(joint_file_impendance_.c_str(), fstream::out|ios_base::trunc);

	for (Joint joint : joint_path_impendance_)
	{
		file_joint_impendance
			<< joint.jVal[0] << ","
			<< joint.jVal[1] << ","
			<< joint.jVal[2] << ","
			<< joint.jVal[3] << ","
			<< joint.jVal[4] << ","
			<< joint.jVal[5]
			<< std::endl;
	}
	file_joint_impendance.close();
}

void RecordPath::getCartPath(ur_data_type::CartPose pose, int impendance_count)
{
	/* 处理第一个轨迹点 */
	if (cart_path_.empty())
	{
		cart_path_.push_back(pose);
		cart_path_impendance_.push_back(pose);

		cart_pose_last_ = pose;
	}

	double distance = getP2PDistance(cart_pose_last_.point, pose.point);
	if (distance > record_limit_.cart_p2p_distance_min)
	{
		scaleRpyDistanceP2P(cart_pose_last_.rpy, pose.rpy);
		cart_path_.push_back(pose); // 存入轨迹复现轨迹

		if (impendance_count_ > 5)
		{
			cart_path_impendance_.push_back(pose); // 存入阻抗复现轨迹
		}

		cart_pose_last_ = pose;
	}
}

void RecordPath::recorCartPath(std::string file)
{
	std::ofstream file_cart(cart_pose_file_.c_str(), fstream::out|ios_base::trunc);
	for (CartPose pose : cart_path_)
	{
		file_cart
			<< pose.point.x << ","
			<< pose.point.y << ","
			<< pose.point.z << ","
			<< pose.rpy.rx << ","
			<< pose.rpy.ry << ","
			<< pose.rpy.rz
			<< std::endl;
	}
	file_cart.close();
}

void RecordPath::recordCartPathImpendance(std::string file)
{
	std::ofstream file_impendance(cart_pose_file_impendance_.c_str(), ios::out|ios::trunc);
	for (CartPose pose : cart_path_impendance_)
	{
		file_impendance
			<< pose.point.x << ","
			<< pose.point.y << ","
			<< pose.point.z << ","
			<< pose.rpy.rx << ","
			<< pose.rpy.ry << ","
			<< pose.rpy.rz
			<< std::endl;
	}
	file_impendance.close();
}

bool RecordPath::addEndPositionInJointPath(Joint joint)
{
#ifdef SET_RECORD_LIMIT_MAX
	std::array<double, 6> position;
	getLastJointPositionFomeFile(joint_file_, position);

	Joint joint_end;
	joint_end.jVal[0] = position[0];
	joint_end.jVal[1] = position[1];
	joint_end.jVal[2] = position[2];
	joint_end.jVal[3] = position[3];
	joint_end.jVal[4] = position[4];
	joint_end.jVal[5] = position[5];

	if (maxJointDistanceP2P(joint_end, joint) > P2P_LIMIT_MAX_JOINT)
	{
		printf("distance too large, can not record the position\n");
		return false;
	}
#endif
	std::ofstream file_joint(joint_file_.c_str(), ios::app);
	file_joint
		<< joint.jVal[0] << std::string(",")
		<< joint.jVal[1] << ","
		<< joint.jVal[2] << ","
		<< joint.jVal[3] << ","
		<< joint.jVal[4] << ","
		<< joint.jVal[5] << std::endl;
	file_joint.close();
	return true;
}

bool RecordPath::addEndPositionInJointPathImpendance(Joint joint)
{
#ifdef SET_RECORD_LIMIT_MAX
	std::array<double, 6> position;
	getLastJointPositionFomeFile(joint_file_, position);

	Joint joint_end;
	joint_end.jVal[0] = position[0];
	joint_end.jVal[1] = position[1];
	joint_end.jVal[2] = position[2];
	joint_end.jVal[3] = position[3];
	joint_end.jVal[4] = position[4];
	joint_end.jVal[5] = position[5];

	if (maxJointDistanceP2P(joint_end, joint) > P2P_LIMIT_MAX_JOINT)
	{
		printf("distance too large, can not record the position\n");
		return false;
	}
#endif
	std::ofstream file_joint(joint_file_impendance_.c_str(), ios::app);
	file_joint
		<< joint.jVal[0] << std::string(",")
		<< joint.jVal[1] << ","
		<< joint.jVal[2] << ","
		<< joint.jVal[3] << ","
		<< joint.jVal[4] << ","
		<< joint.jVal[5] << std::endl;
	file_joint.close();
}

bool RecordPath::addEndPositionInCartPath(CartPose pose)
{
#ifdef SET_RECORD_LIMIT_MAX
	std::array<double, 6> position;
	getLastJointPositionFomeFile(cart_pose_file_, position);

	CartPose pose_end = doubleArrayConvertToCartesianPose(position);

	if (getP2PDistance(pose_end.point, pose.point) > P2P_LIMIT_MAX_CART) /* 50mm */
	{
		printf("distance too large, can not record the position\n");
		return false;
	}
#endif
	std::ofstream file_joint(cart_pose_file_.c_str(), ios::app);

	file_joint
		<< pose.point.x << std::string(",")
		<< pose.point.y << std::string(",")
		<< pose.point.z << std::string(",")
		<< pose.rpy.rx <<  std::string(",")
		<< pose.rpy.ry <<  std::string(",")
		<< pose.rpy.rz << std::endl;

	file_joint.close();
}

bool RecordPath::addEndPositionInCartPathImpendance(CartPose pose)
{
#ifdef SET_RECORD_LIMIT_MAX
	std::array<double, 6> position;
	getLastJointPositionFomeFile(cart_pose_file_impendance_, position);

	CartPose pose_end = doubleArrayConvertToCartesianPose(position);

	if (getP2PDistance(pose_end.point, pose.point) > P2P_LIMIT_MAX_CART)
	{
		printf("distance too large, can not record the position\n");
		return false;
	}
#endif
	std::ofstream file_joint(cart_pose_file_impendance_.c_str(), ios::app);
	file_joint
		<< pose.point.x << std::string(",")
		<< pose.point.y << std::string(",")
		<< pose.point.z << std::string(",")
		<< pose.rpy.rx <<  std::string(",")
		<< pose.rpy.ry <<  std::string(",")
		<< pose.rpy.rz << std::endl;

	file_joint.close();
}

bool RecordPath::updateEndPosition()
{
	Joint actual_joint = robot_rt_->getActualJoint();
	addEndPositionInJointPath(actual_joint);
	addEndPositionInJointPathImpendance(actual_joint);

	addEndPositionInCartPath(robot_rt_->getActualToolPose());
	addEndPositionInCartPathImpendance(robot_rt_->getActualToolPose());
	return true;
}

bool RecordPath::getLastJointPositionFomeFile(std::string file, std::array<double, 6> &position)
{
	std::ifstream ifile;
	ifile.open(file, std::ios::in); //ios::in 	打开文件用于读取。

	if (!ifile.is_open())
	{
		printf("GeneratePath: open %s file failed\n", file);
		return false;
	}

	std::string line;
	std::string line_last;

	while (!ifile.eof())
	{
		std::getline(ifile, line);
		if (line == "")
		{
			continue;
		}
		else
		{
			line_last = line;
		}
	} 

	ifile.close();

	/* 提取处理第一个位置数据 */
	int index = line_last.find(',', 0);
	position[0] = atof(line_last.substr(0, index).c_str()); 

	int end_index;
	/* 处理余下的位置数据 */
	for (int i = 1; i != position.size(); ++i)
	{
		end_index = line_last.find(',', index + 1);
		position[i] = atof(line_last.substr(index + 1, end_index - index - 1).c_str());
		index = end_index;
	}
	return true;
}

std::string RecordPath::getName() 
{
    return "RecordPath";
}
