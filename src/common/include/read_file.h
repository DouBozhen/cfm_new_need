#ifndef READ_FILE_H
#define READ_FILE_H

#include <algorithm>
#include <cstring>
#include <fstream>

bool readFile(const char* src_file_path, std::string &ft_sensor_ip, std::string &robot_arm_ip)
{
	std::ifstream in_file;
	in_file.open(src_file_path, std::ios::in); //ios::in 	打开文件用于读取。

	if (!in_file.is_open())
	{
		printf("open %s file failed\n", src_file_path);
		return false;
	}
    in_file.ignore();

	std::string line;
    std::getline(in_file, line, '\n');
    if(line == "")
    {
        printf("read data failed\n");
        return false; 
    }

    std::getline(in_file, line, '\n');
    std::string::iterator end_pos = std::remove(line.begin(), line.end(), ' ');
    line.erase(end_pos, line.end());
    ft_sensor_ip = line;

    std::getline(in_file, line, '\n');
    if(line == "")
    {
        printf("read data failed\n");
        return false; 
    }

    line = "";
    std::getline(in_file, line, '\n');
    end_pos = std::remove(line.begin(), line.end(), ' ');
    line.erase(end_pos, line.end());
    robot_arm_ip = line;

	in_file.close();
	return true;
}

#endif

