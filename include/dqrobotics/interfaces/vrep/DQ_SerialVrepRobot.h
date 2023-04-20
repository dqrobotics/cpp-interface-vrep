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

namespace DQ_robotics
{
class DQ_SerialVrepRobot: public DQ_VrepRobot
{
protected:
    std::vector<std::string> joint_names_;
    std::string base_frame_name_;
    void _initialize_joint_names_from_vrep(const std::string& base_robot_name,
                                           const int &robot_dof);

    DQ_SerialVrepRobot(const std::string& base_robot_name,
                       const int& robot_dof,
                       const std::string& robot_name,
                       const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);

    //For backwards compatibility, to be removed in a future version of dqrobotics
    DQ_SerialVrepRobot(const std::string& base_robot_name,
                       const int& robot_dof,
                       const std::string& robot_name,
                       DQ_VrepInterface* vrep_interface);
public:
    virtual std::vector<std::string> get_joint_names();

    virtual void send_q_to_vrep(const VectorXd &q) override;
    virtual void send_q_target_to_vrep(const VectorXd& q_target);
    virtual VectorXd get_q_from_vrep() override;
};
}



