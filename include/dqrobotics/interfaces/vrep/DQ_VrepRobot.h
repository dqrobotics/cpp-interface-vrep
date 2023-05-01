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
        - [2023/04] Added {get,set}_configuration_space_positions, deprecated {send,get}_q_{to,from}_vrep.
        
- Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
        - Added smart pointers, deprecated raw pointers. 
         (Adapted from DQ_PseudoinverseController.h and DQ_KinematicController.h)
*/

#pragma once
#include<string>
#include<memory>
#include<dqrobotics/robot_modeling/DQ_Kinematics.h>
#include<dqrobotics/interfaces/vrep/DQ_VrepInterface.h>
namespace DQ_robotics
{
class DQ_VrepRobot
{
protected:
    std::vector<std::string> _strsplit(const std::string& str, const char& delimiter);

    std::string robot_name_;

    std::shared_ptr<DQ_VrepInterface> vrep_interface_sptr_;
    std::shared_ptr<DQ_VrepInterface> _get_interface_sptr() const;
    DQ_VrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr);

    //For backwards compatibility reasons, to be removed in a future version of dqrobotics
    DQ_VrepInterface* vrep_interface_;
    DQ_VrepInterface* _get_interface_ptr() const;
    DQ_VrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface);

public:
    virtual ~DQ_VrepRobot() = default;

    virtual void set_configuration_space_positions(const VectorXd& q)  = 0;
    virtual VectorXd get_configuration_space_positions()  = 0;

    //For backwards compatibility, to be removed in a future version of dqrobotics
    [[deprecated("Use set_configuration_space_positions instead")]]
    virtual void send_q_to_vrep(const VectorXd& q);
    [[deprecated("Use get_configuration_space_positions instead")]]
    virtual VectorXd get_q_from_vrep();
};
}


