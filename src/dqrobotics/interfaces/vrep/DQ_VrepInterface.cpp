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

/**
 * @brief This protected method extracts a string vector from a const char* element.
 * @param string The output_string pointer that is required by simxCallScriptFunction.
 * @param size The number of output strings returned by simxCallScriptFunction.
 * @returns a string vector.
 *
 *          Example:
 *          // When _call_script_function is called, the method runs the following:
 *          // char* output_strings;
 *          // simxCallScriptFunction(clientid_, obj_name.c_str(), __remap_script_type(scripttype), function_name.c_str(),
 *          //                              intsize, input_ints_ptr, floatsize, input_floats_ptr, stringsize, input_strings_ptr,
 *          //                              0, nullptr, &outIntCnt, &output_ints, &outFloatCnt, &output_floats,  &outStringCnt,
 *          //                              &output_strings, nullptr, nullptr, __remap_op_mode(opmode));
 *          // Then, we have that
 *
 *          std::vector<std::string>  vec_output_strings = __extract_vector_string_from_char_pointer(output_strings, sizestr);
 */
std::vector<std::string> _extract_vector_string_from_char_pointer(const char *string, const int& size)
{
    std::vector<std::string> output_string;
    if (size<0){
        throw std::range_error("Incorrect size. The size must be higher than zero");
    };
    char c;
    char c_0;
    c = string[0];
    c_0 = c;
    int j=0;
    std::string str;
    //int count=0;
    int words = 0;
    while (words != size)
     {
        c = string[j];
        if (c =='\0')
        {
            //count++;
            if (c_0 != '\0')
            {
               if (words<size)
                {
                 words++;
                 output_string.push_back(str);
                 str = {};
                }
            }
        }
        c_0 = c;
        j++;
        str.push_back(c);
     }

    return output_string;
}

/**
 * @brief This protected method remaps the constant properties DQ_VrepInterface::SCRIPT_TYPES to their equivalent
 *        simxInt script type.
 * @param script_type The constant script type of DQ_VrepInterface::SCRIPT_TYPES.
 * @returns The simxInt script type.
 *
 *              Example: st = __remap_script_type(ST_CHILD);
 *
 */
simxInt __remap_script_type(const DQ_VrepInterface::SCRIPT_TYPES& script_type)
{
    switch (script_type)
    {
    case DQ_VrepInterface::ST_CHILD:
        return sim_scripttype_childscript;
    case DQ_VrepInterface::ST_MAIN:
        return sim_scripttype_mainscript;
    case DQ_VrepInterface::ST_CUSTOMIZATION:
        return sim_scripttype_customizationscript;
    }
    throw std::range_error("Unknown script_type in __remap_script_type");
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


/**
 * @brief This method returns the inertia matrix of an object on the CoppeliaSim scene.
 * @param link name The name of the object from which we want to extract the inertia matrix.
 * @param reference_frame The referece frame ("shape_frame" or "absolute_frame") where the inertia matrix is expressed. (Default: "shape_frame")
 * @param function_name The name of the script function to call in the specified script. (Default: "get_inertia")
 * @param obj_name The name of the object where the script is attached to. (Default: "DQRoboticsApiCommandServer")
 * @returns The inertia matrix.
 *
 *              Example:
 *              // This example assumes that in your CoppeliaSim there is a child script called
 *              // "DQRoboticsApiCommandServer", where is defined the following Lua function:
 *              //
 *              //     function get_inertia(inInts,inFloats,inStrings,inBuffer)
 *              //
 *              //         if #inInts>=1 then
 *              //            IM, matrixSFCOM=sim.getShapeInertia(inInts[1])
 *              //
 *              //             if inStrings[1] == 'absolute_frame' then
 *              //                 matrix0_SF=sim.getObjectMatrix(inInts[1],-1)
 *              //                 M = sim.multiplyMatrices(matrix0_SF,matrixSFCOM)
 *              //                I= {{IM[1],IM[2],IM[3]},
 *              //                     {IM[4],IM[5],IM[6]},
 *              //                     {IM[7],IM[8],IM[9]}}
 *              //                 R_0_COM = {{M[1],M[2],M[3]},
 *              //                           {M[5],M[6],M[7]},
 *              //                            {M[9],M[10],M[11]}}
 *              //                 R_0_COM_T = transpose(R_0_COM)
 *              //                 RIR_T = mat_mult(mat_mult(R_0_COM,I), R_0_COM_T)
 *              //                 RIR_T_v = {RIR_T[1][1], RIR_T[1][2], RIR_T[1][3],
 *              //                            RIR_T[2][1], RIR_T[2][2], RIR_T[2][3],
 *              //                            RIR_T[3][1], RIR_T[3][2], RIR_T[3][3]}
 *              //                 resultInertiaMatrix=RIR_T_v
 *              //             else
 *              //                 resultInertiaMatrix=IM
 *              //             end
 *              //             return {},resultInertiaMatrix,{},''
 *              //         end
 *              //     end
 *              // In addition, it is assumed a FrankaEmikaPanda robot manipulator
 *              // in the Coppelia scene.
 *
 *              DQ_VrepInterface vi;
 *              std::string link = "Franka_link5_resp";
 *              MatrixXd inertia_matrix = vi.get_inertia_matrix(link);
 *              std::cout<<"Inertia_matrix expressed in shape frame:    \n"<<inertia_matrix<<std::endl;
 *              std::cout<<"Inertia_matrix expressed in absolute frame: \n"<<vi.get_inertia_matrix(link, "absolute_frame")<<std::endl;
 *              std::cout<<"Inertia_matrix expressed in absolute frame: \n"<<vi.get_inertia_matrix(link, "absolute_frame","get_inertia")<<std::endl;
 *              std::cout<<"Inertia_matrix expressed in absolute frame: \n"<<vi.get_inertia_matrix(link, "absolute_frame","get_inertia","DQRoboticsApiCommandServer")<<std::endl;
 *
 */
MatrixXd DQ_VrepInterface::get_inertia_matrix(const std::string& link_name, const std::string& reference_frame, const std::string& function_name, const std::string& obj_name)

{           
    struct call_script_data data = _remote_call_script_function(function_name, obj_name, {get_object_handle(link_name)}, {}, {reference_frame});
    if (data.output_floats.size()!= 9){
        throw std::range_error("Error in get_inertia_matrix. Incorrect number of returned values from CoppeliaSim. (Expected: 9)");
    }
    MatrixXd inertia_matrix = MatrixXd(3,3);
    inertia_matrix << data.output_floats[0],data.output_floats[1],data.output_floats[2],
                      data.output_floats[3],data.output_floats[4],data.output_floats[5],
                      data.output_floats[6],data.output_floats[7],data.output_floats[8];
    return inertia_matrix;
}


/**
 * @brief This method returns the center of mass of an object on the CoppeliaSim scene.
 * @param link name The name of the object from which we want to extract the center of mass.
 * @param reference_frame The referece frame ("shape_frame" or "absolute_frame") where the center of mass is expressed. (Default: "shape_frame")
 * @param function_name The name of the script function to call in the specified script. (Default: "get_center_of_mass")
 * @param obj_name The name of the object where the script is attached to. (Default: "DQRoboticsApiCommandServer")
 * @returns The center of mass.
 *
 *              Example:
 *              // This example assumes that in your CoppeliaSim there is a child script called
 *              // "DQRoboticsApiCommandServer", where is defined the following Lua function:
 *              //
 *              //  function get_center_of_mass(inInts,inFloats,inStrings,inBuffer)
 *              //     if #inInts>=1 then
 *              //         IM,matrix_SF_COM=sim.getShapeInertia(inInts[1])
 *              //         matrix0_SF=sim.getObjectMatrix(inInts[1],-1)
 *              //         if inStrings[1] == 'absolute_frame' then
 *              //             resultMatrix = sim.multiplyMatrices(matrix0_SF,matrix_SF_COM)
 *              //         else
 *              //             resultMatrix = matrix_SF_COM
 *              //         end
 *              //         return {},{resultMatrix[4], resultMatrix[8],resultMatrix[12]},{},''
 *              //     end
 *              // end
 *              // In addition, it is assumed a FrankaEmikaPanda robot manipulator
 *              // in the Coppelia scene.
 *
 *              DQ_VrepInterface vi;
 *
 *              std::string link = "Franka_link2_resp";
 *              VectorXd center_of_mass = vi.get_center_of_mass(link);
 *              std::cout<<"Center of mass expressed in shape frame:\n"<<center_of_mass<<std::endl;
 *              std::cout<<"Center of mass expressed in absolute frame"<<vi.get_center_of_mass(link, "absolute_frame")<<std::endl;
 *              std::cout<<"Center of mass expressed in absolute frame"<<vi.get_center_of_mass(link, "absolute_frame", "get_center_of_mass")<<std::endl;
 *              std::cout<<"Center of mass expressed in absolute frame"<<vi.get_center_of_mass(link, "absolute_frame", "get_center_of_mass","DQRoboticsApiCommandServer")<<std::endl;
 *
 */
VectorXd DQ_VrepInterface::get_center_of_mass(const std::string& link_name, const std::string& reference_frame, const std::string& function_name, const std::string& obj_name)
{        
    struct call_script_data data = _remote_call_script_function(function_name, obj_name, {get_object_handle(link_name)}, {}, {reference_frame});

    if (data.output_floats.size() != 3){
        throw std::range_error("Error in get_center_of_mass. Incorrect number of returned values from CoppeliaSim. (Expected: 3)");
    }
    VectorXd center_of_mass = VectorXd(3);
    center_of_mass << data.output_floats[0],data.output_floats[1],data.output_floats[2];
    return center_of_mass;
}


/**
 * @brief This method returns the mass of an object on the CoppeliaSim scene.
 * @param link name. The name of the object from which we want to extract the mass.
 * @param function_name The name of the script function to call in the specified script. (Default: "get_mass")
 * @param obj_name The name of the object where the script is attached to. (Default: "DQRoboticsApiCommandServer")
 * @returns The mass of the object.
 *
 *              Example:
 *              // This example assumes that in your CoppeliaSim there is a child script called
 *              // "DQRoboticsApiCommandServer", where is defined the following Lua function:
 *              //
 *              //  function get_mass(inInts,inFloats,inStrings,inBuffer)
 *              //     local mass = {}
 *              //     if #inInts>=1 then
 *              //         mass =sim.getShapeMass(inInts[1])
 *              //         return {},{mass},{},''
 *              //     end
 *              //  end
 *              // In addition, it is assumed that there is a FrankaEmikaPanda robot manipulator
 *              // in the Coppelia scene.
 *
 *              DQ_VrepInterface vi;
 *              double mass = vi.get_mass("Franka_link2_resp");
 *                 // or    = vi.get_mass("Franka_link2_resp", "get_mass")
 *                 // or    = vi.get_mass("Franka_link2_resp", "get_mass","DQRoboticsApiCommandServer")
 *
 */
double DQ_VrepInterface::get_mass(const std::string& link_name, const std::string& function_name, const std::string& obj_name)

{       
    struct call_script_data data = _remote_call_script_function(function_name, obj_name, {get_object_handle(link_name)}, {}, {});
    if (data.output_floats.size() != 1){
        throw std::range_error("Error in get_center_of mass. Incorrect number of returned values from CoppeliaSim. (Expected: 1)");
    }
    return data.output_floats[0];
}


call_script_data DQ_VrepInterface::remote_call_script_function(const std::string&  function_name, const std::string&  obj_name, const std::vector<int>& input_ints, const std::vector<float>& input_floats, const std::vector<std::string> &input_strings,
                                       const SCRIPT_TYPES& scripttype, const OP_MODES& opmode)
{
    struct call_script_data data = _remote_call_script_function(function_name, obj_name, input_ints, input_floats, input_strings,
                                                                scripttype, opmode);
    return data;
}

/**
 * @brief This protected method calls remotely a CoppeliaSim script function.
 * @param function_name The name of the script function to call in the specified script.
 * @param obj_name The name of the object where the script is attached to.
 * @param input_ints The input integer values.
 * @param input_floats The input float values.
 * @param input_strings The input string values.
 * @param scripttype The type of the script. (Default: ST_CHILD)
 * @param opmode The operation mode. (Default: OP_BLOCKING)
 * @returns a call_script_data structure.
 *
 */
call_script_data DQ_VrepInterface::_remote_call_script_function(const std::string&  function_name, const std::string&  obj_name, const std::vector<int>& input_ints, const std::vector<float>& input_floats, const std::vector<std::string> &input_strings,
                                           const SCRIPT_TYPES& scripttype, const OP_MODES& opmode)
{
    struct call_script_data data;
    int return_code = 1;
    VectorXi  vec_output_ints;
    VectorXf  vec_output_floats;
    std::vector<std::string> vec_output_strings;

    const int stringsize = input_strings.size();
    std::string one_string;
    if (stringsize >0)
    {
        // If there are string inputs, we need to convert them from const std::vector<std::string> to std::string.
        // the input strings that are handed over to the script function.
        // Each string should be terminated with one zero char, e.g. "Hello\0World\0".
        // Can be nullptr if inStringCnt is zero.
        for(int i = 0; i < stringsize; ++i)
            {
            one_string += input_strings[i]+'\0';
            }
    }

    int outFloatCnt;
    float* output_floats;

    int outIntCnt;
    int* output_ints;

    int outStringCnt;
    char* output_strings;

    return_code = simxCallScriptFunction(clientid_, obj_name.c_str(), __remap_script_type(scripttype), function_name.c_str(),
                                         input_ints.size(), input_ints.data(), input_floats.size(), input_floats.data(), stringsize, one_string.data(),
                                         0, nullptr, &outIntCnt, &output_ints, &outFloatCnt, &output_floats,  &outStringCnt,
                                         &output_strings, nullptr, nullptr, __remap_op_mode(opmode));

    data.return_code = return_code;
     if (return_code == simx_return_ok)
     {

         int sizefloat = outFloatCnt;
         int sizeint = outIntCnt;
         int sizestr = outStringCnt;

         if (sizeint >0)
         {
             data.output_ints = Map<VectorXi >(output_ints,sizeint);
         }

         if (sizefloat >0)
         {
             data.output_floats = Map<VectorXf>(output_floats, sizefloat);
         }

         if (sizestr >0)
         {
             vec_output_strings = _extract_vector_string_from_char_pointer(output_strings, sizestr);
             data.output_strings = vec_output_strings;
         }
     }else
     {
         std::cout<<"Remote function call failed. Error: "<<return_code<<std::endl;
     }

    return data;
}
