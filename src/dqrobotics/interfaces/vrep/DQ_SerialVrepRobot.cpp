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
- Murilo M. Marinho        (murilomarinho@ieee.org)
        - Original implementation.
*/

#include<dqrobotics/interfaces/vrep/DQ_SerialVrepRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

/**
 * @brief Constructor of the DQ_SerialVrepRobot class
 *
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 */
DQ_SerialVrepRobot::DQ_SerialVrepRobot(const std::string &base_robot_name, const int &robot_dof, const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_VrepRobot(robot_name, vrep_interface_sptr)
{
    _initialize_joint_names_from_vrep(base_robot_name, robot_dof);
}

/**
 * @brief For backwards compatibility only. Do not use this Constructor.
 */
DQ_SerialVrepRobot::DQ_SerialVrepRobot(const std::string &base_robot_name, const int &robot_dof, const std::string &robot_name, DQ_VrepInterface *vrep_interface):
    DQ_VrepRobot(robot_name, vrep_interface)
{
    _initialize_joint_names_from_vrep(base_robot_name, robot_dof);
}

/**
 * @brief DQ_SerialVrepRobot::get_joint_names obtains the joint names used in CoppeliaSim.
 * @return a vector of strings containing each joint name.
 */
std::vector<std::string> DQ_SerialVrepRobot::get_joint_names()
{
    return joint_names_;
}

/**
 * @brief DQ_SerialVrepRobot::set_configuration_space_positions Sends the current configuration to CoppeliaSim.
 * Note that this calls "set_joint_positions" in the remoteAPI, meaning that it is only suitable
 * for passive joints.
 * @param q the configuration-space vector.
 */
void DQ_SerialVrepRobot::set_configuration_space_positions(const VectorXd &q)
{
    _get_interface_ptr()->set_joint_positions(joint_names_,q);
}

/**
 * @brief DQ_SerialVrepRobot::get_configuration_space_positions Obtains the current configuration from CoppeliaSim.
 * @return the configuration-space vector.
 */
VectorXd DQ_SerialVrepRobot::get_configuration_space_positions()
{
    return _get_interface_ptr()->get_joint_positions(joint_names_);
}

/**
 * @brief DQ_SerialVrepRobot::set_target_configuration_space_positions Sends the current configuration to CoppeliaSim as
 * a target configuration for the joint controllers. It calls "set_target_joint_positions" on CoppeliaSim,
 * meaning that it is only suitable for active joints.
 * @param q_target the configuration-space vector.
 */
void DQ_SerialVrepRobot::set_target_configuration_space_positions(const VectorXd &q_target)
{
    _get_interface_ptr()->set_joint_target_positions(joint_names_,q_target);
}

/**
 * @brief DQ_SerialVrepRobot::get_configuration_space_velocities Obtains the current configuration velocities from CoppeliaSim.
 * @return the configuration-space velocity vector.
 */
VectorXd DQ_SerialVrepRobot::get_configuration_space_velocities()
{
    return _get_interface_ptr()->get_joint_velocities(joint_names_);
}

/**
 * @brief DQ_SerialVrepRobot::set_target_configuration_space_velocities Sends the current configuration velocities to CoppeliaSim as
 * a target configuration velocity for the joint controllers. It calls "set_joint_target_velocities" on CoppeliaSim,
 * meaning that it is only suitable for active joints.
 * @param v_target the configuration-space velocity vector.
 */
void DQ_SerialVrepRobot::set_target_configuration_space_velocities(const VectorXd &v_target)
{
    _get_interface_ptr()->set_joint_target_velocities(joint_names_,v_target);
}

/**
 * @brief DQ_SerialVrepRobot::set_configuration_space_torques ends the current configuration torques to CoppeliaSim.
 *  It calls "set_joint_torques" on CoppeliaSim, meaning that it is only suitable for active joints.
 * @param t the configuration-space torque vector.
 */
void DQ_SerialVrepRobot::set_configuration_space_torques(const VectorXd &t)
{
    _get_interface_ptr()->set_joint_torques(joint_names_,t);
}

/**
 * @brief DQ_SerialVrepRobot::get_configuration_space_torques Obtains the current configuration torques from CoppeliaSim.
 * @return the configuration-space torque vector.
 */
VectorXd DQ_SerialVrepRobot::get_configuration_space_torques()
{
    return _get_interface_ptr()->get_joint_torques(joint_names_);
}


/**
 * @brief Initialize joint_names_ from CoppeliaSim considering the conventional naming for serially connected robots.
 * If needed, this should be overriden.
 *
 * @param base_robot_name The base robot name, that is, without any numbering modifier so that it can be checked for issues.
 * Example, if the name of the robot is "Franka#0", then the base name is "Franka".
 */
void DQ_SerialVrepRobot::_initialize_joint_names_from_vrep(const std::string &base_robot_name,
                                                           const int& robot_dof)
{
  std::vector<std::string> splited_name = _strsplit(robot_name_,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string(base_robot_name)) != 0)
    {
        throw std::runtime_error(std::string("Expected ") + base_robot_name);
    }

    std::string robot_index("");
    if(splited_name.size() > 1)
        robot_index = "#"+splited_name[1];

    for(int i=1;i<(robot_dof+1);i++)
    {
        std::string current_joint_name = robot_label + std::string("_joint") + std::to_string(i) + robot_index;
        joint_names_.push_back(current_joint_name);
    }
    base_frame_name_ = joint_names_[0];
}

/**
 * For backwards compatibility only. Do not use this method.
 */
void DQ_SerialVrepRobot::send_q_target_to_vrep(const VectorXd &q_target)
{
    set_target_configuration_space_velocities(q_target);
}

}
