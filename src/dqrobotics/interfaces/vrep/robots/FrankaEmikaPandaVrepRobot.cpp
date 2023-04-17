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

#include<dqrobotics/interfaces/vrep/robots/FrankaEmikaPandaVrepRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>
#include<dqrobotics/robots/FrankaEmikaPandaRobot.h>

namespace DQ_robotics
{

/**
 * @brief Constructor of the FrankaEmikaPandaVrepRobot class
 *
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 */
FrankaEmikaPandaVrepRobot::FrankaEmikaPandaVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_VrepRobot(robot_name, vrep_interface_sptr)
{
  _set_names();
}


/**
 * @brief _set_names sets the joint_names_ and the base_frame_name_ attributes.
 *
 * @param robot_name The name of robot used on the vrep scene.
 */
void FrankaEmikaPandaVrepRobot::_set_names()
{
  std::vector<std::string> splited_name = strsplit(robot_name_,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string("Franka")) != 0)
    {
        throw std::runtime_error("Expected Franka");
    }

    std::string robot_index("");
    if(splited_name.size() > 1)
        robot_index = "#"+splited_name[1];

    for(int i=1;i<8;i++)
    {
        std::string current_joint_name = robot_label + std::string("_joint") + std::to_string(i) + robot_index;
        joint_names_.push_back(current_joint_name);
    }
    base_frame_name_ = joint_names_[0];
}

DQ_SerialManipulatorMDH FrankaEmikaPandaVrepRobot::kinematics()
{
    DQ_SerialManipulatorMDH kin = FrankaEmikaPandaRobot::kinematics();
    kin.set_reference_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));
    kin.set_base_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));

    return kin;
}

void FrankaEmikaPandaVrepRobot::send_q_to_vrep(const VectorXd &q)
{
    _get_interface_ptr()->set_joint_positions(joint_names_,q);
}

void FrankaEmikaPandaVrepRobot::send_q_target_to_vrep(const VectorXd &q_target)
{
    _get_interface_ptr()->set_joint_target_positions(joint_names_,q_target);
}

VectorXd FrankaEmikaPandaVrepRobot::get_q_from_vrep()
{
    return _get_interface_ptr()->get_joint_positions(joint_names_);
}

}
