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
        - [2023/04] Changed the inheritance to DQ_SerialVrepRobot from DQ_VrepRobot.
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
    DQ_SerialVrepRobot("Franka",7,robot_name, vrep_interface_sptr)
{

}

DQ_SerialManipulatorMDH FrankaEmikaPandaVrepRobot::kinematics()
{
    DQ_SerialManipulatorMDH kin = FrankaEmikaPandaRobot::kinematics();
    kin.set_reference_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));
    kin.set_base_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));

    return kin;
}

}
