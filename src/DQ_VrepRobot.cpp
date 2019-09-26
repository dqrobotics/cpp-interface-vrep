#include<dqrobotics/interfaces/DQ_VrepRobot.h>

namespace DQ_robotics
{
DQ_VrepRobot::DQ_VrepRobot(const std::string& robot_name, VrepInterface* vrep_interface)
{
    robot_name_ = robot_name;
    if(vrep_interface == nullptr)
        throw std::runtime_error("Null reference to vrep_interface, initialize it first!");
    vrep_interface_ = vrep_interface;
}

//Very simple strsplit
std::vector<std::string> DQ_VrepRobot::strsplit(const std::string& str, const char& delimiter)
{
    std::vector<std::string> string_vector;

    std::string current_string("");
    for(char const &c: str)
    {
        if(c == delimiter)
        {
            string_vector.push_back(current_string);
            current_string = std::string("");
        }
        else
            current_string.push_back(c);
    }
    string_vector.push_back(current_string);

    return string_vector;
}
}
