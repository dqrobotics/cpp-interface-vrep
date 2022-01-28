/**
(C) Copyright 2022 DQ Robotics Developers

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

#include <dqrobotics/interfaces/vrep/DQ_VrepInterface.h>

#include"extApi.h"
#include"simConst.h"

#include<thread>
#include<chrono>

///****************************************************************************************
///                        PRIVATE FUNCTIONS
/// ***************************************************************************************

void DQ_VrepInterface::__insert_or_update_map(const std::string &objectname, const DQ_VrepInterfaceMapElement &element)
{
    auto ret = name_to_element_map_.insert ( std::pair<std::string,DQ_VrepInterfaceMapElement>(objectname,element));
    if (ret.second==false) {
        name_to_element_map_.at(objectname)=element;
    }
}

int DQ_VrepInterface::__get_handle_from_map(const std::string &objectname)
{
    if(name_to_element_map_.count(objectname)==1)
    {
        return name_to_element_map_.at(objectname).get_handle();
    }
    else
        return get_object_handle(objectname);
}

DQ_VrepInterfaceMapElement& DQ_VrepInterface::__get_element_from_map(const std::string &objectname)
{
    //Update map if needed
    __get_handle_from_map(objectname);

    if(name_to_element_map_.count(objectname)==1)
    {
        return name_to_element_map_.at(objectname);
    }
    else
        throw std::runtime_error("Unexpected error @ __get_element_from_map");
}

std::string __simx_int_to_string(const simxInt& ret)
{
    std::string output("");
    //http://www.coppeliarobotics.com/helpFiles/en/remoteApiConstants.htm
    if(ret & simx_return_novalue_flag)
        output += " {No Value Error} ";
    if(ret & simx_return_timeout_flag)
        output += " {Timeout Error} ";
    if(ret & simx_return_illegal_opmode_flag)
        output += " {Illegal Opmode} ";
    if(ret & simx_return_remote_error_flag)
        output += " {Remote Error}";
    if(ret & simx_error_split_progress_flag)
        output += " {Split Progress}";
    if(ret & simx_error_local_error_flag)
        output += " {Local Error}";
    if(ret & simx_error_initialize_error_flag)
        output += " {Initialize Error}";

    return output;
}

void __retry_function(const std::function<simxInt(void)> &f, const int& MAX_TRY_COUNT, const int& TIMEOUT_IN_MILISECONDS, std::atomic_bool* no_blocking_loops, const DQ_VrepInterface::OP_MODES& opmode)
{
    int retry_counter = 0;
    int function_result = 0;
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
        if((function_result == simx_error_novalue_flag) && (opmode == DQ_VrepInterface::OP_STREAMING))
        {
            return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(TIMEOUT_IN_MILISECONDS));
        retry_counter++;
    }
    throw std::runtime_error("Timeout in VREP communication. Error: " + __simx_int_to_string(function_result) +".");
}

simxInt __remap_op_mode(const DQ_VrepInterface::OP_MODES& opmode)
{
    switch(opmode)
    {
    case DQ_VrepInterface::OP_BLOCKING:
        return simx_opmode_blocking;
    case DQ_VrepInterface::OP_BUFFER:
        return simx_opmode_buffer;
    case DQ_VrepInterface::OP_ONESHOT:
        return simx_opmode_oneshot;
    case DQ_VrepInterface::OP_STREAMING:
        return simx_opmode_streaming;
    }
    throw std::range_error("Unknown opmode in __remap_op_mode");
}

///****************************************************************************************
///                        PUBLIC FUNCTIONS
/// ***************************************************************************************

DQ_VrepInterface::DQ_VrepInterface(std::atomic_bool* no_blocking_loops)
{
    no_blocking_loops_ = no_blocking_loops;
    global_retry_count_ = 0;
    clientid_ = -1;
    __insert_or_update_map(VREP_OBJECTNAME_ABSOLUTE,DQ_VrepInterfaceMapElement(-1));
}

DQ_VrepInterface::~DQ_VrepInterface()
{
    disconnect();
}

bool DQ_VrepInterface::connect(const int &port, const int& TIMEOUT_IN_MILISECONDS, const int &MAX_TRY_COUNT)
{
    TIMEOUT_IN_MILISECONDS_ = TIMEOUT_IN_MILISECONDS;
    MAX_TRY_COUNT_          = MAX_TRY_COUNT;

    //The timeout for simxStart makes more sense as a negative number
    //http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxStart

    clientid_ = simxStart("127.0.0.1",port,1,1,-TIMEOUT_IN_MILISECONDS_,1);
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

bool DQ_VrepInterface::connect(const std::string &ip, const int &port, const int &TIMEOUT_IN_MILISECONDS, const int &MAX_TRY_COUNT)
{
    TIMEOUT_IN_MILISECONDS_ = TIMEOUT_IN_MILISECONDS;
    MAX_TRY_COUNT_          = MAX_TRY_COUNT;

    //The timeout for simxStart makes more sense as a negative number
    //http://www.coppeliarobotics.com/helpFiles/en/remoteApiFunctions.htm#simxStart
    clientid_ = simxStart(ip.c_str(),port,1,1,-TIMEOUT_IN_MILISECONDS_,1);
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

void DQ_VrepInterface::disconnect()
{
    if(clientid_>-1)
        simxFinish(clientid_);
}

void DQ_VrepInterface::disconnect_all()
{
    simxFinish(-1);
}

void DQ_VrepInterface::start_simulation() const
{
    simxStartSimulation(clientid_,simx_opmode_blocking);
}

void DQ_VrepInterface::stop_simulation() const
{
    simxStopSimulation(clientid_,simx_opmode_blocking);
}

bool DQ_VrepInterface::is_simulation_running() const
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

void DQ_VrepInterface::set_synchronous(const bool &flag)
{
    simxSynchronous(clientid_, flag);
}

void DQ_VrepInterface::trigger_next_simulation_step()
{
    simxSynchronousTrigger(clientid_);
}

int DQ_VrepInterface::wait_for_simulation_step_to_end()
{
    int ping_time;
    simxGetPingTime(clientid_, &ping_time);
    return ping_time;
}

int DQ_VrepInterface::get_object_handle(const std::string &objectname)
{
    int hp;
    const std::function<simxInt(void)> f = std::bind(simxGetObjectHandle,clientid_,objectname.c_str(),&hp,simx_opmode_blocking);
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,OP_BLOCKING);
    ///Updates handle map
    __insert_or_update_map(objectname,DQ_VrepInterfaceMapElement(hp));
    return hp;
}

std::vector<int> DQ_VrepInterface::get_object_handles(const std::vector<std::string>& objectnames)
{
    int n = objectnames.size();
    std::vector<int> handles(n);
    for(int i=0;i<n;i++)
    {
        handles[i]=get_object_handle(objectnames[i]);
    }
    return handles;
}


DQ DQ_VrepInterface::get_object_translation(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    simxFloat tp[3];
    const std::function<simxInt(void)> f = std::bind(simxGetObjectPosition,clientid_,handle,relative_to_handle,tp,__remap_op_mode(opmode));
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    const DQ t(0,tp[0],tp[1],tp[2]);
    return t;
}
DQ DQ_VrepInterface::get_object_translation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return get_object_translation(handle,__get_handle_from_map(relative_to_objectname),opmode);
}
DQ DQ_VrepInterface::get_object_translation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
{
    return get_object_translation(__get_handle_from_map(objectname),relative_to_handle,opmode);
}
DQ DQ_VrepInterface::get_object_translation(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = __get_element_from_map(objectname);
        if(!element.state_from_function_signature(std::string("get_object_translation")))
        {
            get_object_translation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),OP_STREAMING);
        }
        return get_object_translation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),OP_BUFFER);
    }
    else
        return get_object_translation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),opmode);
}


DQ DQ_VrepInterface::get_object_rotation(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    simxFloat rp[4];
    const std::function<simxInt(void)> f = std::bind(simxGetObjectQuaternion,clientid_,handle,relative_to_handle,rp,__remap_op_mode(opmode));
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    const DQ r(rp[3],rp[0],rp[1],rp[2],0,0,0,0);
    return normalize(r); //We need to normalize here because vrep uses 32bit precision and our DQ are 64bit precision.
}
DQ DQ_VrepInterface::get_object_rotation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return get_object_rotation(handle,__get_handle_from_map(relative_to_objectname),opmode);
}
DQ DQ_VrepInterface::get_object_rotation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
{
    return get_object_rotation(__get_handle_from_map(objectname),relative_to_handle,opmode);
}
DQ DQ_VrepInterface::get_object_rotation(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = __get_element_from_map(objectname);
        if(!element.state_from_function_signature(std::string("get_object_rotation")))
        {
            get_object_rotation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),OP_STREAMING);
        }
        return get_object_rotation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),OP_BUFFER);
    }
    else
        return get_object_rotation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),opmode);
}

DQ DQ_VrepInterface::get_object_pose(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    DQ t = get_object_translation(handle,relative_to_handle,opmode);
    DQ r = get_object_rotation(handle,relative_to_handle,opmode);
    DQ h = r+0.5*E_*t*r;
    return h;
}
DQ DQ_VrepInterface::get_object_pose(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return get_object_pose(handle,__get_handle_from_map(relative_to_objectname),opmode);
}
DQ DQ_VrepInterface::get_object_pose(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
{
    return get_object_pose(__get_handle_from_map(objectname),relative_to_handle,opmode);
}
DQ DQ_VrepInterface::get_object_pose(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    DQ t = get_object_translation(objectname,relative_to_objectname,opmode);
    DQ r = get_object_rotation(objectname,relative_to_objectname,opmode);
    DQ h = r+0.5*E_*t*r;
    return h;
}

std::vector<DQ> DQ_VrepInterface::get_object_poses(const std::vector<int> &handles, const int &relative_to_handle, const OP_MODES &opmode)
{
    std::vector<double>::size_type n = handles.size();
    std::vector<DQ> hs(n);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        hs[i]=get_object_pose(handles[i],relative_to_handle,opmode);
    }
    return hs;
}

void DQ_VrepInterface::set_object_translation(const int &handle, const int &relative_to_handle, const DQ& t, const OP_MODES &opmode) const
{
    simxFloat tp[3];
    tp[0]=float(t.q(1));
    tp[1]=float(t.q(2));
    tp[2]=float(t.q(3));

    simxSetObjectPosition(clientid_,handle,relative_to_handle,tp,__remap_op_mode(opmode));
}
void DQ_VrepInterface::set_object_translation(const int& handle, const std::string& relative_to_objectname, const DQ& t, const OP_MODES& opmode)
{
    return set_object_translation(handle,__get_handle_from_map(relative_to_objectname),t,opmode);
}
void DQ_VrepInterface::set_object_translation(const std::string& objectname, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode)
{
    return set_object_translation(__get_handle_from_map(objectname),relative_to_handle,t,opmode);
}
void DQ_VrepInterface::set_object_translation(const std::string& objectname, const DQ& t, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return set_object_translation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),t,opmode);
}

void DQ_VrepInterface::set_object_rotation(const int &handle, const int &relative_to_handle, const DQ& r, const OP_MODES &opmode) const
{
    simxFloat rp[4];
    rp[0]=float(r.q(1));
    rp[1]=float(r.q(2));
    rp[2]=float(r.q(3));
    rp[3]=float(r.q(0));

    simxSetObjectQuaternion(clientid_,handle,relative_to_handle,rp,__remap_op_mode(opmode));
}
void DQ_VrepInterface::set_object_rotation(const int& handle, const std::string& relative_to_objectname, const DQ& r, const OP_MODES& opmode)
{
    return set_object_rotation(handle,__get_handle_from_map(relative_to_objectname),r,opmode);
}
void DQ_VrepInterface::set_object_rotation(const std::string& objectname, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode)
{
    return set_object_rotation(__get_handle_from_map(objectname),relative_to_handle,r,opmode);
}
void DQ_VrepInterface::set_object_rotation(const std::string& objectname, const DQ& r, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return set_object_rotation(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),r,opmode);
}


void DQ_VrepInterface::set_object_pose(const int &handle, const int &relative_to_handle, const DQ& h, const OP_MODES &opmode) const
{
    set_object_translation(handle,relative_to_handle,translation(h),opmode);
    set_object_rotation(handle,relative_to_handle,P(h),opmode);
}
void DQ_VrepInterface::set_object_pose(const int& handle, const std::string& relative_to_objectname, const DQ& h, const OP_MODES& opmode)
{
    return set_object_pose(handle,__get_handle_from_map(relative_to_objectname),h,opmode);
}
void DQ_VrepInterface::set_object_pose(const std::string& objectname, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode)
{
    return set_object_pose(__get_handle_from_map(objectname),relative_to_handle,h,opmode);
}
void DQ_VrepInterface::set_object_pose(const std::string& objectname, const DQ& h, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return set_object_pose(__get_handle_from_map(objectname),__get_handle_from_map(relative_to_objectname),h,opmode);
}

void DQ_VrepInterface::set_object_poses(const std::vector<int> &handles, const int &relative_to_handle, const std::vector<DQ> &hs, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_object_pose(handles[i],relative_to_handle,hs[i],opmode);
    }
}


double DQ_VrepInterface::get_joint_position(const int &handle, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f;
    const std::function<simxInt(void)> f = std::bind(simxGetJointPosition,clientid_,handle,&angle_rad_f,__remap_op_mode(opmode));
    __retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    return double(angle_rad_f);
}

double DQ_VrepInterface::get_joint_position(const std::string& jointname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = __get_element_from_map(jointname);
        if(!element.state_from_function_signature(std::string("get_joint_position")))
        {
            get_joint_position(element.get_handle(),OP_STREAMING);
        }
        return get_joint_position(element.get_handle(),OP_BUFFER);
    }
    else
        return get_joint_position(__get_handle_from_map(jointname),opmode);
}

void DQ_VrepInterface::set_joint_position(const int &handle, const double &angle_rad, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f = simxFloat(angle_rad);
    simxSetJointPosition(clientid_,handle,angle_rad_f,__remap_op_mode(opmode));
}
void DQ_VrepInterface::set_joint_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode)
{
    return set_joint_position(__get_handle_from_map(jointname),angle_rad,opmode);
}

void DQ_VrepInterface::set_joint_target_position(const int &handle, const double &angle_rad, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f = simxFloat(angle_rad);
    simxSetJointTargetPosition(clientid_,handle,angle_rad_f,__remap_op_mode(opmode));
}
void DQ_VrepInterface::set_joint_target_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode)
{
    return set_joint_target_position(__get_handle_from_map(jointname),angle_rad,opmode);
}

VectorXd DQ_VrepInterface::get_joint_positions(const std::vector<int> &handles, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    VectorXd joint_positions(n);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        joint_positions(i)=get_joint_position(handles[i],opmode);
    }
    return joint_positions;
}

VectorXd DQ_VrepInterface::get_joint_positions(const std::vector<std::string> &jointnames, const OP_MODES &opmode)
{
    std::vector<double>::size_type n = jointnames.size();
    VectorXd joint_positions(n);
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        joint_positions(i)=get_joint_position(jointnames[i],opmode);
    }
    //simxPauseSimulation(clientid_,0);
    return joint_positions;
}


void DQ_VrepInterface::set_joint_positions(const std::vector<int> &handles, const VectorXd &angles_rad, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_position(handles[i],angles_rad(i),opmode);
    }
}

void DQ_VrepInterface::set_joint_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad, const OP_MODES &opmode)
{
    if(jointnames.size() != angles_rad.size())
    {
        throw std::runtime_error("Incompatible sizes in set_joint_positions");
    }
    std::vector<double>::size_type n = jointnames.size();
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_position(jointnames[i],angles_rad(i),opmode);
    }
    //simxPauseSimulation(clientid_,0);
}

void DQ_VrepInterface::set_joint_target_positions(const std::vector<int> &handles, const VectorXd &angles_rad, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_target_position(handles[i],angles_rad(i),opmode);
    }
}

void DQ_VrepInterface::set_joint_target_positions(const std::vector<std::string> &jointnames, const VectorXd &angles_rad, const OP_MODES &opmode)
{
    if(int(jointnames.size()) != int(angles_rad.size()))
    {
        throw std::runtime_error("Incompatible sizes in set_joint_positions");
    }
    std::vector<double>::size_type n = jointnames.size();
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_target_position(jointnames[i],angles_rad(i),opmode);
    }
    //simxPauseSimulation(clientid_,0);
}

void DQ_VrepInterface::start_video_recording()
{
    const unsigned char video_recording_state = 1;
    simxSetBooleanParameter(clientid_,sim_boolparam_video_recording_triggered,video_recording_state,__remap_op_mode(OP_ONESHOT));
}

void DQ_VrepInterface::stop_video_recording()
{
    const unsigned char video_recording_state = 0;
    simxSetBooleanParameter(clientid_,sim_boolparam_video_recording_triggered,video_recording_state,__remap_op_mode(OP_ONESHOT));
}

bool DQ_VrepInterface::is_video_recording()
{
    unsigned char video_recording_state;
    simxGetBooleanParameter(clientid_,sim_boolparam_video_recording_triggered,&video_recording_state,__remap_op_mode(OP_BLOCKING));
    return static_cast<bool>(video_recording_state);
}


