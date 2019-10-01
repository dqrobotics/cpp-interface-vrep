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

#ifndef DQ_ROBOTICS_LBR4P_VREP_ROBOT_HEADER_GUARD
#define DQ_ROBOTICS_LBR4P_VREP_ROBOT_HEADER_GUARD

#include <vector>
#include <dqrobotics/interfaces/DQ_VrepRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulator.h>

namespace DQ_robotics
{
class LBR4pVrepRobot: public DQ_VrepRobot
{
private:
    std::vector<std::string> joint_names_;
    std::string base_frame_name_;
public:
    LBR4pVrepRobot(const std::string& robot_name, VrepInterface* vrep_interface);

    void send_q_to_vrep(const VectorXd &q) override;
    VectorXd get_q_from_vrep() override;

    DQ_SerialManipulator kinematics();
};
}


#endif
