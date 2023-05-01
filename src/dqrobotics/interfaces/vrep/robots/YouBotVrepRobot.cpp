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

#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/robot_modeling/DQ_HolonomicBase.h>
#include<dqrobotics/interfaces/vrep/robots/YouBotVrepRobot.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

YouBotVrepRobot::YouBotVrepRobot(const std::string& robot_name, DQ_VrepInterface* vrep_interface): DQ_VrepRobot(robot_name, vrep_interface)
{
    adjust_ = ((cos(pi/2) + i_*sin(pi/2)) * (cos(pi/4) + j_*sin(pi/4)))*(1+0.5*E_*-0.1*k_);
    _set_names(robot_name);
}


/**
 * @brief Constructor of the YouBotVrepRobot class
 * 
 * @param robot_name The name of robot used on the vrep scene.
 * @param vrep_interface_sptr The DQ_VrepInterface smart pointer.
 * 
 *               Example:
 *               auto vi = std::make_shared<DQ_VrepInterface>(DQ_VrepInterface());
 *               vi->connect(19997,100,5);
 *               vi->start_simulation();
 *               YouBotVrepRobot youbot_vreprobot("youBot", vi);
 *               auto q = youbot_vreprobot.get_q_from_vrep();
 *               vi->stop_simulation();
 *               vi->disconnect();
 */
YouBotVrepRobot::YouBotVrepRobot(const std::string& robot_name, const std::shared_ptr<DQ_VrepInterface>& vrep_interface_sptr):DQ_VrepRobot(robot_name, vrep_interface_sptr)
{
    adjust_ = ((cos(pi/2) + i_*sin(pi/2)) * (cos(pi/4) + j_*sin(pi/4)))*(1+0.5*E_*-0.1*k_);
    _set_names(robot_name);
}


/**
 * @brief _set_names sets the joint_names_ and the base_frame_name_ attributes.
 * 
 * @param robot_name The name of robot used on the vrep scene.
 */
void YouBotVrepRobot::_set_names(const std::string& robot_name)
{
    std::vector<std::string> splited_name = _strsplit(robot_name_,'#');
    std::string robot_label = splited_name[0];

    if(robot_label.compare(std::string("youBot")) != 0)
    {
        throw std::runtime_error("Expected youBot");
    }

    std::string robot_index("");
    if(splited_name.size() > 1)
        robot_index = splited_name[1];

    for(int i=0;i<5;i++)
    {
        std::string current_joint_name = robot_label + std::string("ArmJoint") + std::to_string(i) + robot_index;
        joint_names_.push_back(current_joint_name);
    }
    base_frame_name_ = std::string("youBot");
}

DQ_robotics::DQ_SerialWholeBody YouBotVrepRobot::kinematics()
{
    const double pi2 = pi/2.0;

    Matrix<double,5,5> dh(5,5);
    dh <<    0,      pi2,       0,      pi2,    0,
            0.147,    0,       0,        0,    0.218,
            0.033,    0.155,   0.135,    0,    0,
            pi2,      0,       0,      pi2,    0,
            0, 0, 0, 0, 0;


    auto arm = std::make_shared<DQ_SerialManipulatorDH>(DQ_SerialManipulatorDH(dh));
    auto base = std::make_shared<DQ_HolonomicBase>(DQ_HolonomicBase());

    DQ x_bm = 1 + E_*0.5*(0.165*i_ + 0.11*k_);

    base->set_frame_displacement(x_bm);

    DQ_SerialWholeBody kin(std::static_pointer_cast<DQ_Kinematics>(base));
    kin.add(std::static_pointer_cast<DQ_Kinematics>(arm));

    DQ effector = 1 + E_*0.5*0.3*k_;
    kin.set_effector(effector);

    return kin;
}

void YouBotVrepRobot::set_configuration_space_positions(const VectorXd &q)
{
    double x = q(0);
    double y = q(1);
    double phi = q(2);

    DQ r = cos(phi/2.0)+k_*sin(phi/2.0);
    DQ p = x * i_ + y * j_;
    DQ pose = (1 +E_*0.5*p)*r;

    _get_interface_ptr()->set_joint_positions(joint_names_,q.tail<5>());
    _get_interface_ptr()->set_object_pose(base_frame_name_, pose * conj(adjust_));
}

VectorXd YouBotVrepRobot::get_configuration_space_positions()
{
    DQ base_x = _get_interface_ptr()->get_object_pose(base_frame_name_) * adjust_;
    VectorXd base_t = vec4(translation(base_x));
    double base_phi = rotation_angle(rotation(base_x));
    VectorXd base_arm_q = _get_interface_ptr()->get_joint_positions(joint_names_);
    VectorXd q(8);
    q << base_t(1), base_t(2), base_phi, base_arm_q;
    return q;
}
}
