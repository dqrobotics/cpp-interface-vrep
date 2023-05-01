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
        
- Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
        - Added smart pointers, deprecated raw pointers. 
         (Adapted from DQ_PseudoinverseController.h and DQ_KinematicController.h)
*/

#include<dqrobotics/interfaces/vrep/DQ_VrepRobot.h>

namespace DQ_robotics
{
//Very simple strsplit
std::vector<std::string> DQ_VrepRobot::_strsplit(const std::string& str, const char& delimiter)
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


/**
 * For backwards compatibility only. Do not use this Constructor.
 */
DQ_VrepRobot::DQ_VrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface)
{
    robot_name_ = robot_name;
    if(vrep_interface == nullptr)
        throw std::runtime_error("Null reference to vrep_interface, initialize it first!");
    vrep_interface_ = vrep_interface;
}

/**
 * For backwards compatibility only. Do not use this method.
 */
void DQ_VrepRobot::send_q_to_vrep(const VectorXd &q)
{
    set_configuration_space_positions(q);
}

/**
 * For backwards compatibility only. Do not use this method.
 */
VectorXd DQ_VrepRobot::get_q_from_vrep()
{
    return get_configuration_space_positions();
}


/**
 * @brief Constructor of the DQ_VrepRobot class
 * 
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 */
DQ_VrepRobot::DQ_VrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr)
{
    robot_name_ = robot_name;
    if(!vrep_interface_sptr)
        throw std::runtime_error("Null reference to vrep_interface, initialize it first!");
    vrep_interface_sptr_ = vrep_interface_sptr;
}


/**
 * @brief _get_interface_ptr returns the DQ_VrepInterface raw pointer
 * @returns The desired raw pointer.
 */
DQ_VrepInterface *DQ_VrepRobot::_get_interface_ptr() const
{
    return vrep_interface_sptr_ ? vrep_interface_sptr_.get() : vrep_interface_;
}

/**
 * @brief _get_interface_sptr returns the DQ_VrepInterface smart pointer
 * @returns The desired smart pointer.
 */
std::shared_ptr<DQ_VrepInterface> DQ_VrepRobot::_get_interface_sptr() const
{
    if(!vrep_interface_sptr_)
        throw std::runtime_error("DQ_VrepRobot::_get_interface_sptr invalid interface pointer");
    return vrep_interface_sptr_;   
}

}
