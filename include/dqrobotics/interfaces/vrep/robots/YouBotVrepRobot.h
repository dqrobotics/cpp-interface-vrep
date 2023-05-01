/**
(C) Copyright 2019-2023 DQ Robotics Developers

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
- Murilo M. Marinho        (murilomarinho@ieee.org)
        - Responsible for the original implementation.
        - [2023/04] Adjusted class to use the new {set,get}_configuration_space_positions.
        
- Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
        - Added smart pointers, deprecated raw pointers. 
         (Adapted from DQ_PseudoinverseController.h and DQ_KinematicController.h)
*/

#pragma once
#include <memory>
#include <string>
#include <vector>
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialWholeBody.h>

namespace DQ_robotics
{
class YouBotVrepRobot: public DQ_VrepRobot
{
private:
    std::vector<std::string> joint_names_;
    std::string base_frame_name_;
    DQ adjust_;
    void _set_names(const std::string& robot_name);
public:
    [[deprecated("Use the smart pointer version instead")]]
    YouBotVrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface);
    YouBotVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);

    void set_configuration_space_positions(const VectorXd &q) override;
    VectorXd get_configuration_space_positions() override;

    DQ_SerialWholeBody kinematics();
};
}



