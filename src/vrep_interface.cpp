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

#include <dqrobotics/interfaces/vrep_interface.h>

#include"extApi.h"
#include"v_repConst.h"

#include<unistd.h>
#include<thread>
#include<chrono>
#include<iostream>

///****************************************************************************************
///                        PRIVATE FUNCTIONS
/// ***************************************************************************************

void VrepInterface::__insert_or_update_map(const std::string &objectname, const int &handle)
{
    std::pair<std::map<std::string,int>::iterator,bool> ret;
    ret = name_to_handle_map_.insert ( std::pair<std::string,int>(objectname,handle));
    if (ret.second==false) {
        name_to_handle_map_.at(objectname)=handle;
    }
}

int VrepInterface::__get_handle_from_map(const std::string &objectname)
{
    if(name_to_handle_map_.count(objectname)==1)
    {
        return name_to_handle_map_.at(objectname);
    }
    else
        return get_object_handle(objectname);
}

void __retry_function(const std::function<simxInt(void)> &f, const int& MAX_TRY_COUNT, const int& TIMEOUT_IN_MILISECONDS, std::atomic_bool* no_blocking_loops)
{
    int retry_counter = 0;
    int function_result;
    while(retry_counter<MAX_TRY_COUNT)
    {
        if(no_blocking_loops!=nullptr)
        {
            if(*no_blocking_loops)
                return;
        }
        function_result = f();
        if(function_result == simx_return_ok)
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_IN_MILISECONDS));
        retry_counter++;
    }
    throw std::runtime_error("Timeout in VREP communication. Error: " + std::to_string(function_result) +".");
}

simxInt __remap_op_mode(const VrepInterface::OP_MODES& opmode)
{
    switch(opmode)
    {
    case VrepInterface::OP_BLOCKING:
        return simx_opmode_blocking;
    case VrepInterface::OP_BUFFER:
        return simx_opmode_buffer;
    case VrepInterface::OP_ONESHOT:
        return simx_opmode_oneshot;
    case VrepInterface::OP_STREAMING:
        return simx_opmode_streaming;
    }
    throw std::range_error("Unknown opmode in __remap_op_mode");
}

///****************************************************************************************
///                        PUBLIC FUNCTIONS
/// ***************************************************************************************

VrepInterface::VrepInterface(std::atomic_bool* no_blocking_loops)
{
    no_blocking_loops_ = no_blocking_loops;
    global_retry_count_ = 0;
    std::cout << "****************************************" << std::endl;
    std::cout << "VrepInterface LGPLv3 Murilo@NML (c) 2018" << std::endl;
    std::cout << "****************************************" << std::endl;
    clientid_ = -1;
    __insert_or_update_map(VREP_OBJECTNAME_ABSOLUTE,-1);
}

bool VrepInterface::connect(const int&port, const int& TIMEOUT_IN_MILISECONDS, const int &MAX_TRY_COUNT)
{
    clientid_ = simxStart("127.0.0.1",port,true,true,TIMEOUT_IN_MILISECONDS_,5);
    TIMEOUT_IN_MILISECONDS_ = TIMEOUT_IN_MILISECONDS;
    MAX_TRY_COUNT_          = MAX_TRY_COUNT;
    if(clientid_!=-1)
    {
        //We'll start at least one streaming service at connection time so that we can grab the simulation
        //state correctly. http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxGetInMessageInfo
        int dummyValue;
        simxGetIntegerParameter(clientid_,sim_intparam_program_version,&dummyValue,simx_opmode_streaming);
        return true;
    }
    else
        return false;
}

bool VrepInterface::connect(const std::string &ip, const int &port, const int &TIMEOUT_IN_MILISECONDS, const int &MAX_TRY_COUNT)
{
    clientid_ = simxStart(ip.c_str(),port,true,true,TIMEOUT_IN_MILISECONDS_,5);
    TIMEOUT_IN_MILISECONDS_ = TIMEOUT_IN_MILISECONDS;
    MAX_TRY_COUNT_          = MAX_TRY_COUNT;
    if(clientid_!=-1)
    {
        //We'll start at least one streaming service at connection time so that we can grab the simulation
        //state correctly. http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxGetInMessageInfo
        int dummyValue;
        simxGetIntegerParameter(clientid_,sim_intparam_program_version,&dummyValue,simx_opmode_streaming);
        return true;
    }
    else
        return false;
}

void VrepInterface::disconnect()
{
    if(clientid_>-1)
        simxFinish(clientid_);
}

void VrepInterface::disconnect_all()
{
    simxFinish(-1);
}

void VrepInterface::start_simulation() const
{
    simxStartSimulation(clientid_,simx_opmode_blocking);
}

void VrepInterface::stop_simulation() const
{
    simxStopSimulation(clientid_,simx_opmode_blocking);
}

bool VrepInterface::is_simulation_running() const
{
    simxInt simulation_state;
    simxGetInMessageInfo(clientid_,simx_headeroffset_server_state,&simulation_state);
    if(simulation_state & 0x1)
    {
        return true;
    }
    else
    {
        return false;
    }
}

int VrepInterface::get_object_handle(const std::string &objectname)
{
    int hp;
    const std::function<simxInt(void)> f = std::bind(simxGetObjectHandle,clientid_,objectname.c_str(),&hp,simx_opmode_blocking);
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_);
    ///Updates handle map
    __insert_or_update_map(objectname,hp);
    return hp;
}

std::vector<int> VrepInterface::get_object_handles(const std::vector<std::string>& objectnames)
{
    int n = objectnames.size();
    std::vector<int> handles(n);
    for(int i=0;i<n;i++)
    {
        handles[i]=get_object_handle(objectnames[i]);
    }
    return handles;
}

DQ VrepInterface::get_object_translation(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    simxFloat tp[3];
    const std::function<simxInt(void)> f = std::bind(simxGetObjectPosition,clientid_,handle,relative_to_handle,tp,__remap_op_mode(opmode));
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_);
    DQ t(0,tp[0],tp[1],tp[2]);
    return t;
}

DQ VrepInterface::get_object_rotation(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    simxFloat rp[4];
    const std::function<simxInt(void)> f = std::bind(simxGetObjectQuaternion,clientid_,handle,relative_to_handle,rp,__remap_op_mode(opmode));
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_);
    DQ r(rp[3],rp[0],rp[1],rp[2],0,0,0,0);
    return normalize(r);
}

DQ VrepInterface::get_object_pose(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    DQ t = get_object_translation(handle,relative_to_handle,opmode);
    DQ r = get_object_rotation(handle,relative_to_handle,opmode);
    DQ h = r+0.5*E_*t*r;
    return h;
}

std::vector<DQ> VrepInterface::get_object_poses(const std::vector<int> &handles, const int &relative_to_handle, const OP_MODES &opmode)
{
    int n = handles.size();
    std::vector<DQ> hs(n);
    for(int i=0;i<n;i++)
    {
        hs[i]=get_object_pose(handles[i],relative_to_handle,opmode);
    }
    return hs;
}

void VrepInterface::set_object_translation(const int &handle, const int &relative_to_handle, const DQ& t, const OP_MODES &opmode) const
{
    simxFloat tp[3];
    tp[0]=t.q(1);
    tp[1]=t.q(2);
    tp[2]=t.q(3);

    simxSetObjectPosition(clientid_,handle,relative_to_handle,tp,__remap_op_mode(opmode));
}

void VrepInterface::set_object_rotation(const int &handle, const int &relative_to_handle, const DQ& r, const OP_MODES &opmode) const
{
    simxFloat rp[4];
    rp[0]=r.q(1);
    rp[1]=r.q(2);
    rp[2]=r.q(3);
    rp[3]=r.q(0);

    simxSetObjectQuaternion(clientid_,handle,relative_to_handle,rp,__remap_op_mode(opmode));
}


void VrepInterface::set_object_pose(const int &handle, const int &relative_to_handle, const DQ& h, const OP_MODES &opmode) const
{
    set_object_translation(handle,relative_to_handle,translation(h),opmode);
    set_object_rotation(handle,relative_to_handle,P(h),opmode);
}

void VrepInterface::set_object_poses(const std::vector<int> &handles, const int &relative_to_handle, const std::vector<DQ> &hs, const OP_MODES &opmode) const
{
    int n = handles.size();
    for(int i=0;i<n;i++)
    {
        set_object_pose(handles[i],relative_to_handle,hs[i],opmode);
    }
}


double VrepInterface::get_joint_position(const int &handle, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f;
    const std::function<simxInt(void)> f = std::bind(simxGetJointPosition,clientid_,handle,&angle_rad_f,__remap_op_mode(opmode));
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_);
    return double(angle_rad_f);
}

void VrepInterface::set_joint_position(const int &handle, const double &angle_rad, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f = simxFloat(angle_rad);
    simxSetJointPosition(clientid_,handle,angle_rad_f,__remap_op_mode(opmode));
}

void VrepInterface::set_joint_target_position(const int &handle, const double &angle_rad, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f = simxFloat(angle_rad);
    simxSetJointTargetPosition(clientid_,handle,angle_rad_f,__remap_op_mode(opmode));
}

VectorXd VrepInterface::get_joint_positions(const std::vector<int> &handles, const OP_MODES &opmode) const
{
    int n = handles.size();
    VectorXd joint_positions(n);
    for(int i=0;i<n;i++)
    {
        joint_positions(i)=get_joint_position(handles[i],opmode);
    }
    return joint_positions;
}

VectorXd VrepInterface::get_joint_positions(const std::vector<std::string> &jointnames, const OP_MODES &opmode)
{
    int n = jointnames.size();
    VectorXd joint_positions(n);
    for(int i=0;i<n;i++)
    {
        joint_positions(i)=get_joint_position(jointnames[i],opmode);
    }
    return joint_positions;
}


void VrepInterface::set_joint_positions(const std::vector<int> &handles, const VectorXd &angles_rad, const OP_MODES &opmode) const
{
    int n = handles.size();
    for(int i=0;i<n;i++)
    {
        set_joint_position(handles[i],angles_rad(i),opmode);
    }
}

void VrepInterface::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad, const OP_MODES &opmode)
{
    int n = jointnames.size();
    for(int i=0;i<n;i++)
    {
        set_joint_position(jointnames[i],angles_rad(i),opmode);
    }
}

void VrepInterface::set_joint_target_positions(const std::vector<int> &handles, const VectorXd &angles_rad, const OP_MODES &opmode) const
{
    int n = handles.size();
    for(int i=0;i<n;i++)
    {
        set_joint_target_position(handles[i],angles_rad(i),opmode);
    }
}

void VrepInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad, const OP_MODES &opmode)
{
    int n = jointnames.size();
    for(int i=0;i<n;i++)
    {
        set_joint_target_position(jointnames[i],angles_rad(i),opmode);
    }
}


