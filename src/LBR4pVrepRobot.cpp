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

#include<dqrobotics/interfaces/LBR4pVrepRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

LBR4pVrepRobot::LBR4pVrepRobot(const std::string& robot_name, VrepInterface* vrep_interface): DQ_VrepRobot(robot_name, vrep_interface)
{
    std::vector<std::string> splited_name = strsplit(robot_name_,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string("LBR4p")) != 0)
    {
        std::runtime_error("Expected LBR4p");
    }

    std::string robot_index("");
    if(splited_name.size() > 1)
        robot_index = splited_name[1];

    for(int i=1;i<8;i++)
    {
        std::string current_joint_name = robot_label + std::string("_joint") + std::to_string(i) + robot_index;
        joint_names_.push_back(current_joint_name);
    }
    base_frame_name_ = joint_names_[0];

}

DQ_robotics::DQ_SerialManipulator LBR4pVrepRobot::kinematics()
{
    const double pi2 = pi/2.0;

    Matrix<double,4,7> dh(4,7);
    dh <<  0,     0,     0,   0,   0,    0,   0,
                 0.200, 0,     0.4, 0,   0.39, 0,   0,
                 0,     0,     0,   0,   0,    0,   0,
                 pi2,   -pi2,  pi2,-pi2, pi2, -pi2, 0;
    DQ_SerialManipulator kin(dh,"standard");

    kin.set_reference_frame(vrep_interface_->get_object_pose(base_frame_name_));
    kin.set_base_frame(vrep_interface_->get_object_pose(base_frame_name_));
    kin.set_effector(1+0.5*E_*k_*0.07);

    return kin;
}

void LBR4pVrepRobot::send_q_to_vrep(const VectorXd &q)
{
    vrep_interface_->set_joint_positions(joint_names_,q);
}

VectorXd LBR4pVrepRobot::get_q_from_vrep()
{
    return vrep_interface_->get_joint_positions(joint_names_);
}
}
