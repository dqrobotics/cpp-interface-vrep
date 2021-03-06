﻿/**
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

    /**
     * @brief vrepInterface
     * Default constructor
     */
    DQ_VrepInterface(std::atomic_bool *no_blocking_loops=nullptr);

    /**
     * @brief connect
     * Connects to the VREP remote api server.
     * Calling this function is required before anything else can happen.
     * @param port
     * @param TIMEOUT_IN_MILISECONDS
     * @param MAX_TRY_COUNT
     * @return
     */
    bool connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    bool connect(const std::string& ip, const int& port, const int& TIMEOUT_IN_MILISECONDS, const int& MAX_TRY_COUNT);
    /**
     * @brief disconnect
     * Call this after the last use of this object, or the clientid_ used
     * in this session might become unusable in the future.
     */
    void disconnect();
    /**
     * @brief disconnectAll
     * Tries disconnecting all remote API clients. Be careful with this.
     */
    void disconnect_all();

    /**
     * @brief startSimulation
     * Starts VREP simulation.
     */
    void start_simulation() const;
    /**
     * @brief stopSimulation
     * Stops VREP simulation
     */
    void stop_simulation()  const;

    bool is_simulation_running() const;

    void set_synchronous(const bool& flag);
    void trigger_next_simulation_step();
    int wait_for_simulation_step_to_end();

    /**
     * @brief getObjectHandle
     * Gets an object handle, using VREP_OP_BLOCKING
     * @param objectname
     * @return
     */
    int              get_object_handle(const std::string& objectname);
    /**
     * @brief getObjectHandles
     * A simpler way to call getObjectHandle with several objectnames.
     * @param objectnames
     * @return
     */
    std::vector<int> get_object_handles(const std::vector<std::string>& objectnames);

    /**
     * @brief getObjectTranslation
     * Gets the translation of an object as a quaternion.
     * @param handle
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    DQ   get_object_translation(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_translation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode);
    DQ   get_object_translation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_translation(const std::string& objectname, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_AUTOMATIC);

    /**
     * @brief setObjectTranslation
     * Sets the translation of an object as a given quaternion.
     * @param handle
     * @param relative_to_handle
     * @param t
     * @param opmode
     */
    void set_object_translation(const int& handle, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode) const;
    void set_object_translation(const int& handle, const std::string& relative_to_objectname, const DQ& t, const OP_MODES& opmode);
    void set_object_translation(const std::string& objectname, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode);
    void set_object_translation(const std::string& objectname, const DQ& t, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_ONESHOT);

    /**
     * @brief getObjectRotation
     * Gets the object rotation as a quaternion.
     * @param handle
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    DQ   get_object_rotation(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_rotation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode);
    DQ   get_object_rotation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode);
    DQ   get_object_rotation(const std::string& objectname, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_AUTOMATIC);

    /**
     * @brief setObjectRotation
     * Sets the object rotation as a given quaternion.
     * @param handle
     * @param relative_to_handle
     * @param r
     * @param opmode
     */
    void set_object_rotation(const int& handle, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode) const;
    void set_object_rotation(const int& handle, const std::string& relative_to_objectname, const DQ& r, const OP_MODES& opmode);
    void set_object_rotation(const std::string& objectname, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode);
    void set_object_rotation(const std::string& objectname, const DQ& r, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_ONESHOT);

    /**
     * @brief getObjectPose
     * Gets the object pose as a dual quaternion.
     * @param handle
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    DQ get_object_pose(const int& handle, const int& relative_to_handle, const OP_MODES& opmode);
    DQ get_object_pose(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode);
    DQ get_object_pose(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode);
    DQ get_object_pose(const std::string& objectname, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_AUTOMATIC);

    /**
     * @brief setObjectPose
     * Sets the object pose as a given dual quaternion.
     * @param handle
     * @param relative_to_handle
     * @param h
     * @param opmode
     */
    void set_object_pose(const int& handle, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode) const;
    void set_object_pose(const int& handle, const std::string& relative_to_objectname, const DQ& h, const OP_MODES& opmode);
    void set_object_pose(const std::string& objectname, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode);
    void set_object_pose(const std::string& objectname, const DQ& h, const std::string& relative_to_objectname=VREP_OBJECTNAME_ABSOLUTE, const OP_MODES& opmode=OP_ONESHOT);

    /**
     * @brief getObjectPoses
     * Calls getObjectPose for a collection of handles.
     * @param handles
     * @param relative_to_handle
     * @param opmode
     * @return
     */
    std::vector<DQ> get_object_poses(const std::vector<int>& handles, const int& relative_to_handle, const OP_MODES& opmode);
    /**
     * @brief setObjectPoses
     * Calls setObjectPose for a collection of handles.
     * @param handles
     * @param relative_to_handle
     * @param hs
     * @param opmode
     */
    void     set_object_poses(const std::vector<int>& handles, const int& relative_to_handle, const std::vector<DQ>& hs, const OP_MODES& opmode) const;

    /**
     * @brief getJointPosition
     * Gets the position of a joint.
     * @param handle
     * @param opmode
     * @return
     */
    double   get_joint_position(const int& handle, const OP_MODES& opmode) const;
    double   get_joint_position(const std::string& jointname, const OP_MODES& opmode=OP_AUTOMATIC);

    /**
     * @brief setJointPosition
     * Sets the position of a joint.
     * @param handle
     * @param angle_rad
     * @param opmode
     */
    void     set_joint_position(const int& handle, const double& angle_rad, const OP_MODES& opmode) const;
    void     set_joint_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode=OP_ONESHOT);

    /**
     * @brief set_joint_target_position
     * Sets the target position of a joint.
     * @param handle
     * @param angle_rad
     * @param opmode
     */
    void     set_joint_target_position(const int& handle, const double& angle_rad, const OP_MODES& opmode) const;
    void     set_joint_target_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode=OP_ONESHOT);

    /**
     * @brief getJointPositions
     * Gets the position of a collection of joints.
     * @param handles
     * @param opmode
     * @return
     */
    VectorXd get_joint_positions(const std::vector<int>& handles, const OP_MODES& opmode) const;
    VectorXd get_joint_positions(const std::vector<std::string>& jointnames, const OP_MODES& opmode=OP_AUTOMATIC);
    /**
     * @brief setJointPositions
     * Sets the positions of a collection of joints.
     * @param handles
     * @param angles_rad
     * @param opmode
     */
    void     set_joint_positions(const std::vector<int>& handles, const VectorXd& angles_rad, const OP_MODES& opmode) const;
    void     set_joint_positions(const std::vector<std::string>& jointnames, const VectorXd& angles_rad, const OP_MODES& opmode=OP_ONESHOT);

    /**
     * @brief set_joint_target_positions
     * Sets the target joint positions of a collection of joints
     * @param handles
     * @param angles_rad
     * @param opmode
     */
    void     set_joint_target_positions(const std::vector<int>& handles, const VectorXd& angles_rad, const OP_MODES& opmode) const;
    void     set_joint_target_positions(const std::vector<std::string>& jointnames, const VectorXd& angles_rad, const OP_MODES& opmode=OP_ONESHOT);

private:
    std::map<std::string,DQ_VrepInterfaceMapElement> name_to_element_map_;

    int MAX_TRY_COUNT_;
    int TIMEOUT_IN_MILISECONDS_;
    int clientid_;
    long int global_retry_count_;
    std::atomic_bool* no_blocking_loops_;

    void __insert_or_update_map(const std::string& objectname, const DQ_VrepInterfaceMapElement& element);

    int __get_handle_from_map(const std::string& objectname);

    DQ_VrepInterfaceMapElement &__get_element_from_map(const std::string& objectname);
};

#endif

