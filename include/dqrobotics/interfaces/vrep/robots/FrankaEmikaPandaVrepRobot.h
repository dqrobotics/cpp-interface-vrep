/**
(C) Copyright 2023 DQ Robotics Developers

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
        - Original implementation.
*/

#pragma once
#include <vector>
#include <dqrobotics/interfaces/vrep/DQ_VrepRobot.h>
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics
{
class FrankaEmikaPandaVrepRobot: public DQ_VrepRobot
{
private:
    std::vector<std::string> joint_names_;
    std::string base_frame_name_;
    void _set_names();
public:
    FrankaEmikaPandaVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);

    void send_q_to_vrep(const VectorXd &q) override;
    void send_q_target_to_vrep(const VectorXd& q_target);
    VectorXd get_q_from_vrep() override;

    DQ_SerialManipulatorMDH kinematics();
};
}



