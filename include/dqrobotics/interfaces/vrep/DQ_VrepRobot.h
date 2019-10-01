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

#ifndef DQ_ROBOTICS_VREP_ROBOT_HEADER_GUARD
#define DQ_ROBOTICS_VREP_ROBOT_HEADER_GUARD

#include<string>

#include<eigen3/Eigen/Dense>

#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
namespace DQ_robotics
{
class DQ_VrepRobot
{
protected:
    std::vector<std::string> strsplit(const std::string& str, const char& delimiter);

    std::string robot_name_;
    //Just an observing pointer, we do not take or share ownership (As implied by the raw pointer)
    DQ_VrepInterface* vrep_interface_;

    DQ_VrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface);
public:
    virtual ~DQ_VrepRobot() = default;
    virtual void send_q_to_vrep(const VectorXd& q) = 0;
    virtual VectorXd get_q_from_vrep() = 0;
};
}
#endif

