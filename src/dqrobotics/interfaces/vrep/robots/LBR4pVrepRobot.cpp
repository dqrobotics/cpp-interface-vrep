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
        - [2023/04] Changed the inheritance to DQ_SerialVrepRobot from DQ_VrepRobot.

- Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
        - Added smart pointers, deprecated raw pointers.
         (Adapted from DQ_PseudoinverseController.h and DQ_KinematicController.h)
*/

#include<dqrobotics/interfaces/vrep/robots/LBR4pVrepRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

/**
 * For backwards compatibility only. Do not use this constructor.
 */
LBR4pVrepRobot::LBR4pVrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface):
    DQ_SerialVrepRobot("LBR4p",
                       7,
                       robot_name,
                       vrep_interface)
{

}


/**
 * @brief Constructor of the LBR4pVrepRobot class
 *
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 *
 *               Example:
 *               auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *               vi->connect(19997,100,5);
 *               vi->start_simulation();
 *               LBR4pVrepRobot lbr4p_vreprobot("LBR4p", vi);
 *               auto q = lbr4p_vreprobot.get_q_from_vrep();
 *               vi->stop_simulation();
 *               vi->disconnect();
 */
LBR4pVrepRobot::LBR4pVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):
    DQ_SerialVrepRobot("LBR4p",
                       7,
                       robot_name,
                       vrep_interface_sptr)
{

}


DQ_SerialManipulatorDH LBR4pVrepRobot::kinematics()
{
    const double pi2 = pi/2.0;

    Matrix<double,5,7> dh(5,7);
    dh <<  0,     0,     0,   0,   0,    0,   0,
            0.200, 0,     0.4, 0,   0.39, 0,   0,
            0,     0,     0,   0,   0,    0,   0,
            pi2,   -pi2,  pi2,-pi2, pi2, -pi2, 0,
            0, 0, 0, 0, 0, 0, 0;
    DQ_SerialManipulatorDH kin(dh);
    kin.set_reference_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));
    kin.set_base_frame(_get_interface_ptr()->get_object_pose(base_frame_name_));
    kin.set_effector(1+0.5*E_*k_*0.07);
    return kin;
}

}
