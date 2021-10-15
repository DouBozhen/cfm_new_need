#include <fstream>
#include <vector>
#include <array>
#include <memory>
#include <string>

#include "math_calculate.h"
#include "generate_path.h"

using namespace std;
using namespace ur_data_type;
using namespace math_calculate;

GeneratePath::GeneratePath()
{
    cart_pose_list_.clear();
	joint_position_list_.clear();
}

bool GeneratePath::generateCartPathFromFile(const char* file)
{
	std::ifstream ifile;
	ifile.open(file, std::ios::in); //ios::in 	打开文件用于读取。

	if (!ifile.is_open())
	{
		printf("GeneratePath: open %s file failed\n", file);
		// cart_pose_list_.clear();
		return false;
	}

	std::string line;
	std::array<double, 6> position;

	while (!ifile.eof())
	{
		std::getline(ifile, line);
		if (line == "") continue;

		/* 提取处理第一个位置数据 */
		int index = line.find(',', 0);
		position[0] = atof(line.substr(0, index).c_str()); 

		int end_index;
		/* 处理余下的位置数据 */
		for (int i = 1; i != position.size(); ++i)
		{
			end_index = line.find(',', index + 1);
			position[i] = atof(line.substr(index + 1, end_index - index - 1).c_str());
			index = end_index;
		}

		cart_pose_list_.push_back(doubleArrayConvertToCartesianPose(position));
	} 

	ifile.close();
	pose_start_ = cart_pose_list_[0];
	pose_end_ = cart_pose_list_[cart_pose_list_.size()-1];

	printf("GeneratePath: read %s file successful\n", file);
	return true;
}

bool GeneratePath::generateJointPathFromFile(const char* file)
{
    std::ifstream ifile;
	ifile.open(file, std::ios::in);//ios::in 打开文件用于读取。

	if (!ifile.is_open())
	{
		printf("GeneratePath: open %s file failed\n", file);
		return false;
	}

	std::string line;
	Joint joint;

	joint_position_list_.clear();
	while (!ifile.eof())
	{
		std::getline(ifile, line);
		if (line == "") continue;

		/* 提取处理第一个位置数据 */
		int index = line.find(',', 0);
		joint.jVal[0] = atof(line.substr(0, index).c_str());
		// printf("joint path: %f, ", joint.jVal[0]);

		int end_index;
		/* 处理余下的位置数据 */
		for (int i = 1; i != 6; ++i)
		{
			end_index = line.find(',', index + 1);
			joint.jVal[i] = atof(line.substr(index + 1, end_index - index - 1).c_str());
			index = end_index;
			//printf("%f, ", joint.jVal[i]);
		}
		//printf("\n");
		joint_position_list_.push_back(joint);
	}

	ifile.close();
	joint_start_ = joint_position_list_[0];
	joint_end_ = joint_position_list_[joint_position_list_.size() - 1];

	printf("GeneratePath: read %s file successful, point size: %d\n", file, joint_position_list_.size());
	return true;
}

