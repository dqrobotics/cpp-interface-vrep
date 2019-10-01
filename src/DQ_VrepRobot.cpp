/**
(C) Copyright 2019 DQ Robotics Developers

This file is part of DQ Robotics.

    DQ Robotics is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    DQ Robotics is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with DQ Robotics.  If not, see <http://www.gnu.org/licenses/>.

Contributors:
- Murilo M. Marinho        (murilo@nml.t.u-tokyo.ac.jp)
*/

#include<dqrobotics/interfaces/vrep/DQ_VrepRobot.h>

namespace DQ_robotics
{
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


DQ_VrepRobot::DQ_VrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface)
{
    robot_name_ = robot_name;
    if(vrep_interface == nullptr)
        throw std::runtime_error("Null reference to vrep_interface, initialize it first!");
    vrep_interface_ = vrep_interface;
}
}
