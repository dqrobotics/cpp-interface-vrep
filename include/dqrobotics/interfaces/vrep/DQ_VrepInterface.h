/**
(C) Copyright 2019-2022 DQ Robotics Developers

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
- Juan Jose Quiroz Omana   (juanjqo@g.ecc.u-tokyo.ac.jp)
*/

#ifndef DQ_ROBOTICS_INTERFACE_VREP_HEADER_GUARD
#define DQ_ROBOTICS_INTERFACE_VREP_HEADER_GUARD

#include<atomic>
#include<vector>
#include<functional>
#include<map>
#include<string>

#include<dqrobotics/DQ.h>
#include<dqrobotics/interfaces/vrep/DQ_VrepInterfaceMapElement.h>

const std::string VREP_OBJECTNAME_ABSOLUTE("VREP_OBJECTNAME_ABSOLUTE");

using namespace DQ_robotics;
using namespace Eigen;



class DQ_VrepInterface
{
public:
    enum OP_MODES
    {
        OP_BLOCKING,
        OP_STREAMING,
        OP_ONESHOT,
        OP_BUFFER,
        OP_AUTOMATIC
    };
    enum SCRIPT_TYPES
    {
        ST_CHILD,
        ST_MAIN,
        ST_CUSTOMIZATION
    };
    enum REFERENCE_FRAMES
    {
        BODY_FRAME,
        ABSOLUTE_FRAME
    };


    DQ_VrepInterface(std::atomic_bool *no_blocking_loops=nullptr);


    ~DQ_VrepInterface();


    bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);

    void disconnect();

    void disconnect_all();


    void start_simulation() const;

    void stop_simulation()  const;

    bool is_simulation_running() const;

    void set_synchronous(const bool& flag);
    void trigger_next_simulation_step();
    int wait_for_simulation_step_to_end();


    int              get_object_handle(const std::string& objectname);

    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames);


    DQ   get_object_translation(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_translation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode);
    DQ   get_object_translation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_translation(const std::string& objectname, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_AUTOMATIC);


    void set_object_translation(const int& handle, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode) const;
    void set_object_translation(const int& handle, const std::string& relative_to_objectname, const DQ& t, const OP_MODES& opmode);
    void set_object_translation(const std::string& objectname, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode);
    void set_object_translation(const std::string& objectname, const DQ& t, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_ONESHOT);


    DQ   get_object_rotation(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_rotation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode);
    DQ   get_object_rotation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_rotation(const std::string& objectname, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_AUTOMATIC);


    void set_object_rotation(const int& handle, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode) const;
    void set_object_rotation(const int& handle, const std::string& relative_to_objectname, const DQ& r, const OP_MODES& opmode);
    void set_object_rotation(const std::string& objectname, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode);
    void set_object_rotation(const std::string& objectname, const DQ& r, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_ONESHOT);


    DQ get_object_pose(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ get_object_pose(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode);
    DQ get_object_pose(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode);
    DQ get_object_pose(const std::string& objectname, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_AUTOMATIC);


    void set_object_pose(const int& handle, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode) const;
    void set_object_pose(const int& handle, const std::string& relative_to_objectname, const DQ& h, const OP_MODES& opmode);
    void set_object_pose(const std::string& objectname, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode);
    void set_object_pose(const std::string& objectname, const DQ& h, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_ONESHOT);


    std::vector<DQ> get_object_poses(const std::vector<int>& handles, const int& relative_to_handle, const OP_MODES& opmode);

    void     set_object_poses(const std::vector<int>& handles, const int& relative_to_handle, const std::vector<DQ>& hs, const OP_MODES& opmode) const;

    double   get_joint_position(const int& handle, const OP_MODES& opmode) const;
    double   get_joint_position(const std::string& jointname, const OP_MODES& opmode=OP_AUTOMATIC);

    void     set_joint_position(const int& handle, const double& angle_rad, const OP_MODES& opmode) const;
    void     set_joint_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode=OP_ONESHOT);

    void     set_joint_target_position(const int& handle, const double& angle_rad, const OP_MODES& opmode) const;
    void     set_joint_target_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode=OP_ONESHOT);

    VectorXd get_joint_positions(const std::vector<int>& handles, const OP_MODES& opmode) const;
    VectorXd get_joint_positions(const std::vector<std::string>& jointnames, const OP_MODES& opmode=OP_AUTOMATIC);

    void     set_joint_positions(const std::vector<int>& handles, const VectorXd& angles_rad, const OP_MODES& opmode) const;
    void     set_joint_positions(const std::vector<std::string>& jointnames, const VectorXd& angles_rad, const OP_MODES& opmode=OP_ONESHOT);


    void     set_joint_target_positions(const std::vector<int>& handles, const VectorXd& angles_rad, const OP_MODES& opmode) const;
    void     set_joint_target_positions(const std::vector<std::string>& jointnames, const VectorXd& angles_rad, const OP_MODES& opmode=OP_ONESHOT);

    void start_video_recording();
    void stop_video_recording();
    bool is_video_recording();

    //--New ones------
    void     set_joint_target_velocity(const int& handle, const double& angle_dot_rad, const OP_MODES& opmode) const;
    void     set_joint_target_velocity(const std::string& jointname, const double& angle_dot_rad, const OP_MODES& opmode=OP_ONESHOT);
    void     set_joint_target_velocities(const std::vector<int>& handles, const VectorXd& angles_dot_rad, const OP_MODES& opmode) const;
    void     set_joint_target_velocities(const std::vector<std::string>& jointnames, const VectorXd& angles_dot_rad, const OP_MODES& opmode=OP_ONESHOT);

    double   get_joint_velocity(const int& handle, const OP_MODES& opmode) const;
    double   get_joint_velocity(const std::string& jointname, const OP_MODES& opmode=OP_AUTOMATIC);
    VectorXd get_joint_velocities(const std::vector<int>& handles, const OP_MODES& opmode) const;
    VectorXd get_joint_velocities(const std::vector<std::string>& jointnames, const OP_MODES& opmode=OP_AUTOMATIC);

    void     set_joint_torque(const int& handle, const double& torque, const OP_MODES& opmode) const;
    void     set_joint_torque(const std::string& jointname, const double& torque, const OP_MODES& opmode=OP_ONESHOT);
    void     set_joint_torques(const std::vector<int>& handles, const VectorXd& torques, const OP_MODES& opmode) const;
    void     set_joint_torques(const std::vector<std::string>& jointnames, const VectorXd& torques, const OP_MODES& opmode=OP_ONESHOT);

    double   get_joint_torque(const int& handle, const OP_MODES& opmode) const;
    double   get_joint_torque(const std::string& jointname, const OP_MODES& opmode=OP_AUTOMATIC);
    VectorXd get_joint_torques(const std::vector<int>& handles, const OP_MODES& opmode) const;
    VectorXd get_joint_torques(const std::vector<std::string>& jointnames, const OP_MODES& opmode=OP_AUTOMATIC);

    MatrixXd get_inertia_matrix(const std::string& link_name, const REFERENCE_FRAMES& reference_frame=BODY_FRAME, const std::string& function_name = "get_inertia", const std::string& obj_name= "DQRoboticsApiCommandServer");
    MatrixXd get_inertia_matrix(const int& handle, const REFERENCE_FRAMES& reference_frame=BODY_FRAME, const std::string& function_name = "get_inertia", const std::string& obj_name= "DQRoboticsApiCommandServer");

    DQ      get_center_of_mass(const std::string& link_name, const REFERENCE_FRAMES& reference_frame=BODY_FRAME, const std::string& function_name = "get_center_of_mass", const std::string& obj_name= "DQRoboticsApiCommandServer");
    DQ      get_center_of_mass(const int& handle, const REFERENCE_FRAMES& reference_frame=BODY_FRAME, const std::string& function_name = "get_center_of_mass", const std::string& obj_name= "DQRoboticsApiCommandServer");

    double get_mass(const std::string& link_name, const std::string& function_name = "get_mass", const std::string& obj_name= "DQRoboticsApiCommandServer");
    double get_mass(const int& handle, const std::string& function_name = "get_mass", const std::string& obj_name= "DQRoboticsApiCommandServer");

    int  get_object_parent(int object_handle, const OP_MODES &op_mode = OP_AUTOMATIC);
    int  get_object_parent(const std::string &object_name, const OP_MODES &op_mode = OP_AUTOMATIC);

    void set_object_parent(int object_handle, int parent_object_handle, bool keep_in_place, const OP_MODES &op_mode = OP_AUTOMATIC);
    void set_object_parent(const std::string &object_name, const std::string &parent_object_name, bool keep_in_place, const OP_MODES &op_mode = OP_AUTOMATIC);

    void remove_object_parents(int object_handle, bool keep_in_place, const OP_MODES &op_mode = OP_AUTOMATIC);
    void remove_object_parents(const std::string &object_name, bool keep_in_place, const OP_MODES &op_mode = OP_AUTOMATIC);


private:
    std::map<std::string,DQ_VrepInterfaceMapElement> name_to_element_map_;

    int MAX_TRY_COUNT_;
    int TIMEOUT_IN_MILISECONDS_;
    int clientid_;
    long int global_retry_count_;
    std::atomic_bool* no_blocking_loops_;

    void _insert_or_update_map(const std::string& objectname, const DQ_VrepInterfaceMapElement& element);

    int _get_handle_from_map(const std::string& objectname);

    DQ_VrepInterfaceMapElement &_get_element_from_map(const std::string& objectname);

    int _call_script_function(const std::string&  function_name, const std::string&  obj_name, const std::vector<int>& input_ints, const std::vector<float>& input_floats, const std::vector<std::string> &input_strings,
                                int* outIntCnt, int** output_ints, int* outFloatCnt, float** output_floats, int* outStringCnt, char** output_strings,
                                const SCRIPT_TYPES& scripttype = ST_CHILD, const OP_MODES& opmode = OP_BLOCKING);

};

#endif

