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
  - Initial implementation.
  - 2023/5/15 Added better error description for get_object_handle().
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

/**
 * @brief This custom structure containts the data of the DQ_VrepInterface::call_script_function method.
 * @param return_code The remote API function flag returned. Example: simx_return_ok.
 * @param output_ints The returned integer values.
 * @param output_floats The returned float values.
 * @param output_strings The returned string values.
 *
 */
struct call_script_data
{
    int return_code;
    VectorXi output_ints;
    VectorXf output_floats;
    std::vector<std::string> output_strings;
    //unsigned char retBuffer;

};

void DQ_VrepInterface::_insert_or_update_map(const std::string &objectname, const DQ_VrepInterfaceMapElement &element)
{
    auto ret = name_to_element_map_.insert ( std::pair<std::string,DQ_VrepInterfaceMapElement>(objectname,element));
    if (ret.second==false) {
        name_to_element_map_.at(objectname)=element;
    }
}

int DQ_VrepInterface::_get_handle_from_map(const std::string &objectname)
{
    if(name_to_element_map_.count(objectname)==1)
    {
        return name_to_element_map_.at(objectname).get_handle();
    }
    else
        return get_object_handle(objectname);
}

DQ_VrepInterfaceMapElement& DQ_VrepInterface::_get_element_from_map(const std::string &objectname)
{
    //Update map if needed
    _get_handle_from_map(objectname);

    if(name_to_element_map_.count(objectname)==1)
    {
        return name_to_element_map_.at(objectname);
    }
    else
        throw std::runtime_error("Unexpected error @ _get_element_from_map");
}

std::string _simx_int_to_string(const simxInt& ret)
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

void _retry_function(const std::function<simxInt(void)> &f, const int& MAX_TRY_COUNT, const int& TIMEOUT_IN_MILISECONDS, std::atomic_bool* no_blocking_loops, const DQ_VrepInterface::OP_MODES& opmode)
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
    throw std::runtime_error("Timeout in VREP communication. Error: " + _simx_int_to_string(function_result) +".");
}


/**
 * @brief This protected method remaps the constant properties DQ_VrepInterface::OP_MODES to their equivalent
 *        simx_opmode value.
 * @param opmode The constant operation mode of DQ_VrepInterface::OP_MODES.
 * @returns The simx_opmode value.
 *
 *              Example: op = _remap_op_mode(OP_BLOCKING);
 *
 */
simxInt _remap_op_mode(const DQ_VrepInterface::OP_MODES& opmode)
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
    throw std::range_error("Unknown opmode in _remap_op_mode");
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
 * @brief This method returns a call_script_data structure from elements returned by _call_script_function().
 * @param return_code The return code of the simxCallScriptFunction method.
 * @param outIntCnt (pointer) The number of returned integer values.
 * @param output_ints (pointer) The returned integer values.
 * @param outFloatCnt (pointer) The number of returned floating-point values.
 * @param output_floats (pointer) The returned floating-point values.
 * @param outStringCnt (pointer)  The number of returned strings.
 * @param output_strings (pointer) The returned strings.
 * @returns a call_script_data structure.
 *     Example:
 *        int outIntCnt;
 *        int* output_ints;
 *        int outFloatCnt;
 *        float* output_floats;
 *        int outStringCnt;
 *        char* output_strings;
 *        int return_code = _call_script_function(function_name, obj_name, {}, {}, {},&outIntCnt, &output_ints, &outFloatCnt, &output_floats, &outStringCnt, &output_strings);
 *
 *        //To get the returned values, you can use _extract_call_script_data_from_pointers().
 *        //Example:
 *         call_script_data data = _extract_call_script_data_from_pointers(return_code, outIntCnt, output_ints, outFloatCnt, output_floats, outStringCnt, output_strings);
 *
 *
 */
call_script_data _extract_call_script_data_from_pointers(int return_code, int outIntCnt, int* output_ints, int outFloatCnt, float* output_floats, int outStringCnt, char* output_strings)
{    
    struct call_script_data data;
    data.return_code = return_code;

    if (return_code == simx_return_ok)
    {
        if (outIntCnt >0)
        {
            data.output_ints = Map<VectorXi >(output_ints, outIntCnt);
        }

        if (outFloatCnt >0)
        {
            data.output_floats = Map<VectorXf>(output_floats, outFloatCnt);
        }
        if (outStringCnt >0)
        {
            data.output_strings = _extract_vector_string_from_char_pointer(output_strings, outStringCnt);
        }
    }

    return data;
}


/**
 * @brief This protected method remaps the constant properties DQ_VrepInterface::REFERENCE_FRAMES to their equivalent
 *        string.
 * @param reference_frame The constant reference frame of DQ_VrepInterface::REFERENCE_FRAMES.
 * @returns The string related to the reference frame.
 *
 *              Example: rf = _remap_reference_frame(ABSOLUTE_FRAME);
 *
 */
std::string _remap_reference_frame(const DQ_VrepInterface::REFERENCE_FRAMES& reference_frame)
{
    switch(reference_frame)
    {
    case DQ_VrepInterface::BODY_FRAME:
        return "body_frame";
    case DQ_VrepInterface::ABSOLUTE_FRAME:
        return "absolute_frame";
    }
    throw std::range_error("Unknown reference_frame in _remap_reference_frame");
}

/**
 * @brief This protected method remaps the constant properties DQ_VrepInterface::SCRIPT_TYPES to their equivalent
 *        simxInt script type.
 * @param script_type The constant script type of DQ_VrepInterface::SCRIPT_TYPES.
 * @returns The simxInt script type.
 *
 *              Example: st = _remap_script_type(ST_CHILD);
 *
 */
simxInt _remap_script_type(const DQ_VrepInterface::SCRIPT_TYPES& script_type)
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


/**
 * @brief DQ_VrepInterface
 * Default constructor
 */
DQ_VrepInterface::DQ_VrepInterface(std::atomic_bool* no_blocking_loops)
{
    no_blocking_loops_ = no_blocking_loops;
    global_retry_count_ = 0;
    clientid_ = -1;
    _insert_or_update_map(VREP_OBJECTNAME_ABSOLUTE,DQ_VrepInterfaceMapElement(-1));
}


/**
 * @brief ~DQ_VrepInterface
 * Default desconstructor. Calls disconnect.
 */
DQ_VrepInterface::~DQ_VrepInterface()
{
    disconnect();
}


/**
 * @brief This method connects to the VREP remote api server.
 *        Calling this function is required before anything else can happen.
 * @param port
 * @param TIMEOUT_IN_MILISECONDS
 * @param MAX_TRY_COUNT
 * @return
 */
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


/**
 * @brief This method connects to the VREP remote api server.
 *        Calling this function is required before anything else can happen.
 * @param ip
 * @param port
 * @param TIMEOUT_IN_MILISECONDS
 * @param MAX_TRY_COUNT
 * @returns bool status.
 */
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

/**
 * @brief disconnect
 * Call this after the last use of this object, or the clientid_ used
 * in this session might become unusable in the future.
 */
void DQ_VrepInterface::disconnect()
{
    if(clientid_>-1)
        simxFinish(clientid_);
}


/**
 * @brief This method tries disconnecting all remote API clients. Be careful with this.
 */
void DQ_VrepInterface::disconnect_all()
{
    simxFinish(-1);
}


/**
 * @brief This method starts the VREP simulation.
 *
 */
void DQ_VrepInterface::start_simulation() const
{
    simxStartSimulation(clientid_,simx_opmode_blocking);
}


/**
 * @brief This method stops the VREP simulation.
 *
 */
void DQ_VrepInterface::stop_simulation() const
{
    simxStopSimulation(clientid_,simx_opmode_blocking);
}


/**
 * @brief This method returns the simulation status.
 * @returns The simulation status.
 *
 */
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


/**
 * @brief This method enables or disables the stepped mode for the remote API server service that the client is connected to.
 *        Example:
 *                       DQ_VrepInterface vi;
 *                       vi.connect(19997,100,10);
 *                       vi.set_synchronous(true);
 *
 */
void DQ_VrepInterface::set_synchronous(const bool &flag)
{
    simxSynchronous(clientid_, flag);
}


/**
 * @brief This method sends a synchronization trigger signal to the server, which performs
 *        a simulation step when the synchronous mode is used.
 *        Example:
 *                       DQ_VrepInterface vi;
 *                       vi.connect(19997,100,10);
 *                       vi.set_synchronous(true);
 *                       vi.trigger_next_simulation_step();
 *
 *
 */
void DQ_VrepInterface::trigger_next_simulation_step()
{
    simxSynchronousTrigger(clientid_);
}


/**
 * @brief This method returns the time needed for a command to be sent to the server, executed, and sent back.
 * @returns ping_time
 */
int DQ_VrepInterface::wait_for_simulation_step_to_end()
{
    int ping_time;
    simxGetPingTime(clientid_, &ping_time);
    return ping_time;
}


/**
 * @brief This method returns the handle of an object given its object name.
 * @param objectname The object name.
 * @returns The object handle.
 */
int DQ_VrepInterface::get_object_handle(const std::string &objectname)
{
    int hp;
    const std::function<simxInt(void)> f = std::bind(simxGetObjectHandle,clientid_,objectname.c_str(),&hp,simx_opmode_blocking);

    try
    {
        _retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,OP_BLOCKING);
    }
    catch(const std::runtime_error& e)
    {
        throw std::runtime_error(
                    std::string(e.what())
                    + " "
                    + std::string("The most common source of this error is that the object `")
                    + objectname
                    + std::string("` does not exist in the current scene in CoppeliaSim.")
                    );
    }

    _insert_or_update_map(objectname,DQ_VrepInterfaceMapElement(hp));

    return hp;
}


/**
 * @brief This method returns the handles several objects given their object names.
 * @param objectnames The names of the objects.
 * @returns handles The object handles.
 */
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


/**
 * @brief This method gets the translation of an object.
 * @param handle The handle name.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return t The object translation expressed with respect to relative_to_handle.
 */
DQ DQ_VrepInterface::get_object_translation(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    simxFloat tp[3];
    const std::function<simxInt(void)> f = std::bind(simxGetObjectPosition,clientid_,handle,relative_to_handle,tp,_remap_op_mode(opmode));
    _retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    const DQ t(0,tp[0],tp[1],tp[2]);
    return t;
}


/**
 * @brief This method gets the translation of an object.
 * @param handle The handle name.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 * @return t The object translation expressed with respect to relative_to_objectname.
 */
DQ DQ_VrepInterface::get_object_translation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return get_object_translation(handle,_get_handle_from_map(relative_to_objectname),opmode);
}


/**
 * @brief This method gets the translation of an object.
 * @param objectname The object name.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return t The object translation expressed with respect to relative_to_handle.
 */
DQ DQ_VrepInterface::get_object_translation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
{
    return get_object_translation(_get_handle_from_map(objectname),relative_to_handle,opmode);
}


/**
 * @brief This method gets the translation of an object.
 * @param objectname The object name.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 * @return t The object translation expressed with respect to relative_to_objectname.
 */
DQ DQ_VrepInterface::get_object_translation(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = _get_element_from_map(objectname);
        if(!element.state_from_function_signature(std::string("get_object_translation")))
        {
            get_object_translation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),OP_STREAMING);
        }
        return get_object_translation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),OP_BUFFER);
    }
    else
        return get_object_translation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),opmode);
}


/**
 * @brief This method gets the rotation of an object.
 * @param handle The object handle.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return r The object rotation expressed with respect to relative_to_handle.
 */
DQ DQ_VrepInterface::get_object_rotation(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    simxFloat rp[4];
    const std::function<simxInt(void)> f = std::bind(simxGetObjectQuaternion,clientid_,handle,relative_to_handle,rp,_remap_op_mode(opmode));
    _retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    const DQ r(rp[3],rp[0],rp[1],rp[2],0,0,0,0);
    return normalize(r); //We need to normalize here because vrep uses 32bit precision and our DQ are 64bit precision.
}


/**
 * @brief This method gets the rotation of an object.
 * @param handle The object handle.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 * @return r The object rotation expressed with respect to relative_to_objectname.
 */
DQ DQ_VrepInterface::get_object_rotation(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return get_object_rotation(handle,_get_handle_from_map(relative_to_objectname),opmode);
}


/**
 * @brief This method gets the rotation of an object.
 * @param objectname The object name.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return r The object rotation expressed with respect to relative_to_handle.
 */
DQ DQ_VrepInterface::get_object_rotation(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
{
    return get_object_rotation(_get_handle_from_map(objectname),relative_to_handle,opmode);
}


/**
 * @brief This method gets the rotation of an object.
 * @param objectname The object name.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 * @return r The object rotation expressed with respect to relative_to_objectname.
 */
DQ DQ_VrepInterface::get_object_rotation(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = _get_element_from_map(objectname);
        if(!element.state_from_function_signature(std::string("get_object_rotation")))
        {
            get_object_rotation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),OP_STREAMING);
        }
        return get_object_rotation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),OP_BUFFER);
    }
    else
        return get_object_rotation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),opmode);
}


/**
 * @brief This method gets the pose of an object.
 * @param handle The object handle.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return h The pose of an object expressed with respect to relative_to_handle.
 */
DQ DQ_VrepInterface::get_object_pose(const int &handle, const int &relative_to_handle, const OP_MODES &opmode)
{
    DQ t = get_object_translation(handle,relative_to_handle,opmode);
    DQ r = get_object_rotation(handle,relative_to_handle,opmode);
    DQ h = r+0.5*E_*t*r;
    return h;
}


/**
 * @brief This method gets the pose of an object.
 * @param handle The object handle.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 * @return h The pose of an object expressed with respect to relative_to_objectname.
 */
DQ DQ_VrepInterface::get_object_pose(const int& handle, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return get_object_pose(handle,_get_handle_from_map(relative_to_objectname),opmode);
}


/**
 * @brief This method gets the pose of an object.
 * @param objectname The object name.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return h The pose of an object expressed with respect to relative_to_handle.
 */
DQ DQ_VrepInterface::get_object_pose(const std::string& objectname, const int& relative_to_handle, const OP_MODES& opmode)
{
    return get_object_pose(_get_handle_from_map(objectname),relative_to_handle,opmode);
}


/**
 * @brief This method gets the pose of an object.
 * @param objectname The object name.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 * @return h The pose of an object expressed with respect to relative_to_objectname.
 */
DQ DQ_VrepInterface::get_object_pose(const std::string& objectname, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    DQ t = get_object_translation(objectname,relative_to_objectname,opmode);
    DQ r = get_object_rotation(objectname,relative_to_objectname,opmode);
    DQ h = r+0.5*E_*t*r;
    return h;
}


/**
 * @brief This method gets the poses of a collection of objects.
 * @param handles The handles of the objects.
 * @param relative_to_handle The reference handle.
 * @param opmode The operation mode.
 * @return hs The poses of a collection of objects expressed with respect to relative_to_handle.
 */
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


/**
 * @brief This method sets the object translation given by a pure quaternion.
 * @param handle The object handle.
 * @param relative_to_handle The reference handle.
 * @param t The desired translation expressed with respect to relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_translation(const int &handle, const int &relative_to_handle, const DQ& t, const OP_MODES &opmode) const
{
    simxFloat tp[3];
    tp[0]=float(t.q(1));
    tp[1]=float(t.q(2));
    tp[2]=float(t.q(3));

    simxSetObjectPosition(clientid_,handle,relative_to_handle,tp,_remap_op_mode(opmode));
}


/**
 * @brief This method sets the object translation given by a pure quaternion.
 * @param handle The object handle.
 * @param relative_to_objectname The name of the reference object.
 * @param t The desired translation expressed with respect to relative_to_objectname.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_translation(const int& handle, const std::string& relative_to_objectname, const DQ& t, const OP_MODES& opmode)
{
    return set_object_translation(handle,_get_handle_from_map(relative_to_objectname),t,opmode);
}


/**
 * @brief This method sets the object translation given by a pure quaternion.
 * @param objectname The object name.
 * @param relative_to_handle The reference handle.
 * @param t The desired translation expressed with respect to relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_translation(const std::string& objectname, const int& relative_to_handle, const DQ& t, const OP_MODES& opmode)
{
    return set_object_translation(_get_handle_from_map(objectname),relative_to_handle,t,opmode);
}


/**
 * @brief This method sets the object translation given by a pure quaternion.
 * @param objectname The object name.
 * @param t The desired translation expressed with respect to relative_to_objectname.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_translation(const std::string& objectname, const DQ& t, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return set_object_translation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),t,opmode);
}


/**
 * @brief This method sets the object rotation given by a unit quaternion.
 * @param handle The object handle.
 * @param relative_to_handle The reference handle.
 * @param r The desired rotation expressed with respect to relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_rotation(const int &handle, const int &relative_to_handle, const DQ& r, const OP_MODES &opmode) const
{
    simxFloat rp[4];
    rp[0]=float(r.q(1));
    rp[1]=float(r.q(2));
    rp[2]=float(r.q(3));
    rp[3]=float(r.q(0));

    simxSetObjectQuaternion(clientid_,handle,relative_to_handle,rp,_remap_op_mode(opmode));
}


/**
 * @brief This method sets the object rotation given by a unit quaternion.
 * @param handle The object handle.
 * @param relative_to_objectname The name of the reference object.
 * @param r The desired rotation expressed with respect to relative_to_objectname.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_rotation(const int& handle, const std::string& relative_to_objectname, const DQ& r, const OP_MODES& opmode)
{
    return set_object_rotation(handle,_get_handle_from_map(relative_to_objectname),r,opmode);
}


/**
 * @brief This method sets the object rotation given by a unit quaternion.
 * @param objectname The object name.
 * @param relative_to_handle The reference handle.
 * @param r The desired rotation expressed with respect to relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_rotation(const std::string& objectname, const int& relative_to_handle, const DQ& r, const OP_MODES& opmode)
{
    return set_object_rotation(_get_handle_from_map(objectname),relative_to_handle,r,opmode);
}


/**
 * @brief This method sets the object rotation given by a unit quaternion.
 * @param objectname The object name.
 * @param r The desired rotation expressed with respect to relative_to_objectname.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_rotation(const std::string& objectname, const DQ& r, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return set_object_rotation(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),r,opmode);
}


/**
 * @brief This method sets the pose of an object.
 * @param handle The object handle.
 * @param relative_to_handle The reference handle.
 * @param h The desired pose expressed with respect to relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_pose(const int &handle, const int &relative_to_handle, const DQ& h, const OP_MODES &opmode) const
{
    set_object_translation(handle,relative_to_handle,translation(h),opmode);
    set_object_rotation(handle,relative_to_handle,P(h),opmode);
}


/**
 * @brief This method sets the pose of an object.
 * @param handle The object handle.
 * @param relative_to_objectname The name of the reference object.
 * @param h The desired pose is expressed with respect to relative_to_objectname.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_pose(const int& handle, const std::string& relative_to_objectname, const DQ& h, const OP_MODES& opmode)
{
    return set_object_pose(handle,_get_handle_from_map(relative_to_objectname),h,opmode);
}


/**
 * @brief This method sets the pose of an object.
 * @param objectname The object name.
 * @param relative_to_handle The reference handle.
 * @param h The desired pose expressed with respect to relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_pose(const std::string& objectname, const int& relative_to_handle, const DQ& h, const OP_MODES& opmode)
{
    return set_object_pose(_get_handle_from_map(objectname),relative_to_handle,h,opmode);
}

/**
 * @brief This method sets the pose of an object.
 * @param objectname The object name.
 * @param h The desired pose expressed with respect to relative_to_objectname.
 * @param relative_to_objectname The name of the reference object.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_pose(const std::string& objectname, const DQ& h, const std::string& relative_to_objectname, const OP_MODES& opmode)
{
    return set_object_pose(_get_handle_from_map(objectname),_get_handle_from_map(relative_to_objectname),h,opmode);
}


/**
 * @brief This method sets the poses of collection of objects.
 * @param handles The object handles.
 * @param relative_to_handle The reference handle.
 * @param hs The desired poses expressed with respect to the relative_to_handle.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_object_poses(const std::vector<int> &handles, const int &relative_to_handle, const std::vector<DQ> &hs, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_object_pose(handles[i],relative_to_handle,hs[i],opmode);
    }
}


/**
 * @brief This method gets the joint position.
 * @param handle The handle of the joint.
 * @param opmode The operation mode.
 * @returns angle_rad_f The joint position.
 */
double DQ_VrepInterface::get_joint_position(const int &handle, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f;
    const std::function<simxInt(void)> f = std::bind(simxGetJointPosition,clientid_,handle,&angle_rad_f,_remap_op_mode(opmode));
    _retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    return double(angle_rad_f);
}


/**
 * @brief This method gets the joint position.
 * @param jointname The name of the joint.
 * @param opmode The operation mode.
 * @returns angle_rad_f The joint position.
 */
double DQ_VrepInterface::get_joint_position(const std::string& jointname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = _get_element_from_map(jointname);
        if(!element.state_from_function_signature(std::string("get_joint_position")))
        {
            get_joint_position(element.get_handle(),OP_STREAMING);
        }
        return get_joint_position(element.get_handle(),OP_BUFFER);
    }
    else
        return get_joint_position(_get_handle_from_map(jointname),opmode);
}


/**
 * @brief This method sets the joint position.
 * @param handle The handle of the joint.
 * @param angles_rad The desired joint position.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_position(const int &handle, const double &angle_rad, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f = simxFloat(angle_rad);
    simxSetJointPosition(clientid_,handle,angle_rad_f,_remap_op_mode(opmode));
}


/**
 * @brief This method sets the joint position.
 * @param jointname The name of the joint.
 * @param angles_rad The desired joint position.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode)
{
    return set_joint_position(_get_handle_from_map(jointname),angle_rad,opmode);
}


/**
 * @brief This method sets the target joint position.
 * @param handle The handle of the joint.
 * @param angles_rad The target joint position.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_target_position(const int &handle, const double &angle_rad, const OP_MODES &opmode) const
{
    simxFloat angle_rad_f = simxFloat(angle_rad);
    simxSetJointTargetPosition(clientid_,handle,angle_rad_f,_remap_op_mode(opmode));
}


/**
 * @brief This method sets the target joint position.
 * @param jointname The name of the joint.
 * @param angles_rad The target joint position.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_target_position(const std::string& jointname, const double& angle_rad, const OP_MODES& opmode)
{
    return set_joint_target_position(_get_handle_from_map(jointname),angle_rad,opmode);
}


/**
 * @brief This method gets the joint positions.
 * @param handle The handles of the joints.
 * @param opmode The operation mode.
 * @returns joint_positions The joint positions.
 */
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


/**
 * @brief This method gets the joint positions.
 * @param jointnames The names of the joints.
 * @param opmode The operation mode.
 * @returns joint_positions The joint positions.
 */
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


/**
 * @brief This method sets the joint positions.
 * @param handles The handles of the joints.
 * @param angles_rad The desired joint position vector.
 * @param opmode  The operation mode.
 */
void DQ_VrepInterface::set_joint_positions(const std::vector<int> &handles, const VectorXd &angles_rad, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_position(handles[i],angles_rad(i),opmode);
    }
}


/**
 * @brief This method sets the joint positions.
 * @param jointnames The names of the joints.
 * @param angles_rad The desired joint position vector.
 * @param opmode  The operation mode.
 */
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


/**
 * @brief This method sets the target joint positions.
 * @param handles The handles of the joints.
 * @param angles_rad The target joint position vector.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_target_positions(const std::vector<int> &handles, const VectorXd &angles_rad, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_target_position(handles[i],angles_rad(i),opmode);
    }
}


/**
 * @brief This method sets the target joint positions.
 * @param jointnames The names of the joints.
 * @param angles_rad The target joint position vector.
 * @param opmode The operation mode.
 *
 *        Example:
 *           std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2","Franka_joint3", "Franka_joint4",
 *                                                  "Franka_joint5", "Franka_joint6","Franka_joint7"};
 *           DQ_VrepInterface vi;
 *           VectorXd u = VectorXd::Zero(7);
 *           vi.set_joint_target_positions(jointnames, u);
 *
 */
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


/**
 * @brief This method starts the video recording.
 */
void DQ_VrepInterface::start_video_recording()
{
    const unsigned char video_recording_state = 1;
    simxSetBooleanParameter(clientid_,sim_boolparam_video_recording_triggered,video_recording_state,_remap_op_mode(OP_ONESHOT));
}


/**
 * @brief This method stops the video recording.
 */
void DQ_VrepInterface::stop_video_recording()
{
    const unsigned char video_recording_state = 0;
    simxSetBooleanParameter(clientid_,sim_boolparam_video_recording_triggered,video_recording_state,_remap_op_mode(OP_ONESHOT));
}


/**
 * @brief This method returns the video recording status.
 */
bool DQ_VrepInterface::is_video_recording()
{
    unsigned char video_recording_state;
    simxGetBooleanParameter(clientid_,sim_boolparam_video_recording_triggered,&video_recording_state,_remap_op_mode(OP_BLOCKING));
    return static_cast<bool>(video_recording_state);
}

// New ones

/**
 * @brief This method sets the joint velocity.
 * @param handle The handle of the joint.
 * @param angle_dot_rad The target angular velocity.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_target_velocity(const int &handle, const double &angle_dot_rad, const OP_MODES &opmode) const
{
    simxFloat angle_dot_rad_f = simxFloat(angle_dot_rad);
    simxSetJointTargetVelocity(clientid_,handle,angle_dot_rad_f,_remap_op_mode(opmode));
}

/**
 * @brief This method sets the joint velocity.
 * @param jointname The name of the joint.
 * @param angle_dot_rad The target angular velocity.
 * @param opmode The operation mode. (Default: OP_ONESHOT)
 */
void DQ_VrepInterface::set_joint_target_velocity(const std::string& jointname, const double& angle_dot_rad, const OP_MODES& opmode)
{
    return set_joint_target_velocity(_get_handle_from_map(jointname), angle_dot_rad,opmode);
}


/**
 * @brief This method sets the joint velocities.
 * @param handle The handles of the joints.
 * @param angles_dot_rad The target angular velocities.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_target_velocities(const std::vector<int> &handles, const VectorXd &angles_dot_rad, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_target_velocity(handles[i],angles_dot_rad(i),opmode);
    }
}


/**
 * @brief This method sets the joint velocities.
 * @param jointnames The names of the joints.
 * @param angles_dot_rad The target angular velocities.
 * @param opmode The operation mode. (Default: OP_ONESHOT)
 *
 *       Example:
 *           std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2","Franka_joint3", "Franka_joint4",
 *                                                  "Franka_joint5", "Franka_joint6","Franka_joint7"};
 *           DQ_VrepInterface vi;
 *           VectorXd u = VectorXd::Zero(7);
 *           vi.set_joint_target_velocities(jointnames, u);
 *
 */
void DQ_VrepInterface::set_joint_target_velocities(const std::vector<std::string> &jointnames, const VectorXd &angles_dot_rad, const OP_MODES &opmode)
{
    if(int(jointnames.size()) != int(angles_dot_rad.size()))
    {
        throw std::runtime_error("Incompatible sizes in set_joint_target_velocities");
    }
    std::vector<double>::size_type n = jointnames.size();
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_target_velocity(jointnames[i],angles_dot_rad(i),opmode);
    }
    //simxPauseSimulation(clientid_,0);
}


/**
 * @brief This method gets the joint velocity.
 * @param handle The handle of the joint.
 * @param opmode The operation mode.
 * @returns angle_dot_rad_f The joint velocity.
 */
double DQ_VrepInterface::get_joint_velocity(const int &handle, const OP_MODES &opmode) const
{
    simxFloat angle_dot_rad_f;
    const std::function<simxInt(void)> f = std::bind(simxGetObjectFloatParameter, clientid_, handle, 2012, &angle_dot_rad_f,_remap_op_mode(opmode));
    _retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    return double(angle_dot_rad_f);
}


/**
 * @brief This method gets the joint velocity.
 * @param jointname The name of the joint.
 * @param opmode The operation mode.
 * @returns angle_dot_rad_f The joint velocity.
 */
double DQ_VrepInterface::get_joint_velocity(const std::string& jointname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = _get_element_from_map(jointname);
        if(!element.state_from_function_signature(std::string("get_joint_velocity")))
        {
            get_joint_velocity(element.get_handle(),OP_STREAMING);
        }
        return get_joint_velocity(element.get_handle(),OP_BUFFER);
    }
    else
        return get_joint_velocity(_get_handle_from_map(jointname),opmode);
}


/**
 * @brief This method gets the joint velocities.
 * @param handle The handles of the joints.
 * @param opmode The operation mode.
 * @returns joint_velocities The joint velocities.
 */
VectorXd DQ_VrepInterface::get_joint_velocities(const std::vector<int> &handles, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    VectorXd joint_velocities(n);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        joint_velocities(i)=get_joint_velocity(handles[i],opmode);
    }
    return joint_velocities;
}


/**
 * @brief This method gets the joint velocities.
 * @param jointnames The names of the joints.
 * @param opmode The operation mode.
 * @returns joint_velocities The joint velocities.
 *
 *      Example:
 *           std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2","Franka_joint3", "Franka_joint4",
 *                                                  "Franka_joint5", "Franka_joint6","Franka_joint7"};
 *           DQ_VrepInterface vi;
 *           std::cout<< "q_dot: "<<vi.get_joint_velocities(jointnames)<<std::endl;
 *
 */
VectorXd DQ_VrepInterface::get_joint_velocities(const std::vector<std::string> &jointnames, const OP_MODES &opmode)
{
    std::vector<double>::size_type n = jointnames.size();
    VectorXd joint_velocities(n);
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        joint_velocities(i)=get_joint_velocity(jointnames[i],opmode);
    }
    //simxPauseSimulation(clientid_,0);
    return joint_velocities;
}


/**
 * @brief This method sets the joint torque.
 * @param handle The handle of the joint.
 * @param torque The torque.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_torque(const int &handle, const double &torque, const OP_MODES &opmode) const
{
    simxFloat torque_f = simxFloat(torque);
    simxFloat angle_dot_rad_max = 10000.0;
    if (torque_f==0)
    {
        angle_dot_rad_max = 0.0;
    }else if (torque_f<0)
    {
        angle_dot_rad_max = -10000.0;
    }
    simxSetJointTargetVelocity(clientid_,handle,angle_dot_rad_max,_remap_op_mode(opmode));
    simxSetJointForce(clientid_,handle,abs(torque_f),_remap_op_mode(opmode));
}

/**
 * @brief This method sets the joint torque.
 * @param jointname The name of the joint.
 * @param torque The torque.
 * @param opmode The operation mode. (Default: OP_ONESHOT)
 */
void DQ_VrepInterface::set_joint_torque(const std::string& jointname, const double& torque, const OP_MODES& opmode)
{
    return set_joint_torque(_get_handle_from_map(jointname), torque,opmode);
}


/**
 * @brief This method sets the joint torques.
 * @param handle The handles of the joints.
 * @param torques The torques.
 * @param opmode The operation mode.
 */
void DQ_VrepInterface::set_joint_torques(const std::vector<int> &handles, const VectorXd &torques, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_torque(handles[i],torques(i),opmode);
    }
}


/**
 * @brief This method sets the joint torques.
 * @param jointnames The names of the joints.
 * @param torques The torques.
 * @param opmode The operation mode. (Default: OP_ONESHOT)
 *
 *      Example:
 *           std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2","Franka_joint3", "Franka_joint4",
 *                                                  "Franka_joint5", "Franka_joint6","Franka_joint7"};
 *           DQ_VrepInterface vi;
 *           VectorXd u = VectorXd::Zero(7);
 *           vi.set_joint_torques(jointnames, u);
 */
void DQ_VrepInterface::set_joint_torques(const std::vector<std::string> &jointnames, const VectorXd &torques, const OP_MODES &opmode)
{
    if(int(jointnames.size()) != int(torques.size()))
    {
        throw std::runtime_error("Incompatible sizes in set_joint_torques");
    }
    std::vector<double>::size_type n = jointnames.size();
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        set_joint_torque(jointnames[i],torques(i),opmode);
    }
    //simxPauseSimulation(clientid_,0);
}


/**
 * @brief This method gets the joint torque.
 * @param handle The handle of the joint.
 * @param opmode The operation mode.
 * @returns torque The joint torque.
 */
double DQ_VrepInterface::get_joint_torque(const int &handle, const OP_MODES &opmode) const
{
    simxFloat torque;
    const std::function<simxInt(void)> f = std::bind(simxGetJointForce, clientid_, handle, &torque,_remap_op_mode(opmode));
    _retry_function(f,MAX_TRY_COUNT_,TIMEOUT_IN_MILISECONDS_,no_blocking_loops_,opmode);
    //We change the signal to get a consistent result.
    return double(-1*torque);
}


/**
 * @brief This method gets the joint torque.
 * @param jointname The name of the joint.
 * @param opmode The operation mode.
 * @returns torque The joint torque.
 */
double DQ_VrepInterface::get_joint_torque(const std::string& jointname, const OP_MODES& opmode)
{
    if(opmode == OP_AUTOMATIC)
    {
        DQ_VrepInterfaceMapElement& element = _get_element_from_map(jointname);
        if(!element.state_from_function_signature(std::string("get_joint_torque")))
        {
            get_joint_torque(element.get_handle(),OP_STREAMING);
        }
        return get_joint_torque(element.get_handle(),OP_BUFFER);
    }
    else
        return get_joint_torque(_get_handle_from_map(jointname),opmode);
}


/**
 * @brief This method gets the joint torques.
 * @param handle The handles of the joints.
 * @param opmode The operation mode.
 * @returns joint_torques The joint torques.
 */
VectorXd DQ_VrepInterface::get_joint_torques(const std::vector<int> &handles, const OP_MODES &opmode) const
{
    std::vector<double>::size_type n = handles.size();
    VectorXd joint_torques(n);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        joint_torques(i)=get_joint_velocity(handles[i],opmode);
    }
    return joint_torques;
}


/**
 * @brief This method gets the joint torques.
 * @param jointnames The names of the joints.
 * @param opmode The operation mode.
 * @returns joint_torques The joint torques.
 *
 *      Example:
 *           std::vector<std::string> jointnames = {"Franka_joint1", "Franka_joint2","Franka_joint3", "Franka_joint4",
 *                                                  "Franka_joint5", "Franka_joint6","Franka_joint7"};
 *           DQ_VrepInterface vi;
 *           std::cout<< "torques: "<<vi.get_joint_torques(jointnames)<<std::endl;
 *
 */
VectorXd DQ_VrepInterface::get_joint_torques(const std::vector<std::string> &jointnames, const OP_MODES &opmode)
{
    std::vector<double>::size_type n = jointnames.size();
    VectorXd joint_torques(n);
    //simxPauseSimulation(clientid_,1);
    for(std::vector<double>::size_type i=0;i<n;i++)
    {
        joint_torques(i)=get_joint_torque(jointnames[i],opmode);
    }
    //simxPauseSimulation(clientid_,0);
    return joint_torques;
}

/**
 * @brief This method returns the inertia matrix of an object on the CoppeliaSim scene.
 * @param handle The handle of the object from which we want to extract the inertia matrix.
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
 *              int handle = vi.get_object_handle("Franka_link5_resp")
 *              MatrixXd inertia_matrix = vi.get_inertia_matrix(handle);
 *              std::cout<<"Inertia_matrix expressed in shape frame:    \n"<<inertia_matrix<<std::endl;
 *              std::cout<<"Inertia_matrix expressed in absolute frame: \n"<<vi.get_inertia_matrix(handle, "absolute_frame")<<std::endl;
 *              std::cout<<"Inertia_matrix expressed in absolute frame: \n"<<vi.get_inertia_matrix(handle, "absolute_frame","get_inertia")<<std::endl;
 *              std::cout<<"Inertia_matrix expressed in absolute frame: \n"<<vi.get_inertia_matrix(handle, "absolute_frame","get_inertia","DQRoboticsApiCommandServer")<<std::endl;
 *
 */
MatrixXd DQ_VrepInterface::get_inertia_matrix(const int& handle, const REFERENCE_FRAMES& reference_frame, const std::string& function_name, const std::string& obj_name)
{
    int outFloatCnt;
    float* output_floats;
    int return_code = _call_script_function(function_name, obj_name, {handle}, {}, {_remap_reference_frame(reference_frame)},
                                            0, nullptr, &outFloatCnt, &output_floats,0, nullptr);
    if (return_code != 0)
    {std::cout<<"Remote function call failed. Error: "<<return_code<<std::endl;}
    call_script_data data = _extract_call_script_data_from_pointers(return_code, 0, nullptr, outFloatCnt, output_floats, 0, nullptr);

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
MatrixXd DQ_VrepInterface::get_inertia_matrix(const std::string& link_name, const REFERENCE_FRAMES& reference_frame, const std::string& function_name, const std::string& obj_name)
{    
    return get_inertia_matrix(_get_handle_from_map(link_name), reference_frame, function_name, obj_name);
}


/**
 * @brief This method returns the center of mass of an object on the CoppeliaSim scene.
 * @param handle The handle of the object from which we want to extract the center of mass.
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
 *              int handle = vi.get_object_handle("Franka_link2_resp";
 *              VectorXd center_of_mass = vi.get_center_of_mass(handle);
 *              std::cout<<"Center of mass expressed in shape frame:\n"<<center_of_mass<<std::endl;
 *              std::cout<<"Center of mass expressed in absolute frame"<<vi.get_center_of_mass(handle, "absolute_frame")<<std::endl;
 *              std::cout<<"Center of mass expressed in absolute frame"<<vi.get_center_of_mass(handle, "absolute_frame", "get_center_of_mass")<<std::endl;
 *              std::cout<<"Center of mass expressed in absolute frame"<<vi.get_center_of_mass(handle, "absolute_frame", "get_center_of_mass","DQRoboticsApiCommandServer")<<std::endl;
 *
 */
DQ DQ_VrepInterface::get_center_of_mass(const int& handle,  const REFERENCE_FRAMES& reference_frame, const std::string& function_name, const std::string& obj_name)
{
    int outFloatCnt;
    float* output_floats;
    int return_code = _call_script_function(function_name, obj_name, {handle}, {}, {_remap_reference_frame(reference_frame)},
                                            0, nullptr, &outFloatCnt, &output_floats,0, nullptr);
    if (return_code != 0)
    {std::cout<<"Remote function call failed. Error: "<<return_code<<std::endl;}
    call_script_data data = _extract_call_script_data_from_pointers(return_code, 0, nullptr, outFloatCnt, output_floats, 0, nullptr);

    if (data.output_floats.size() != 3){
        throw std::range_error("Error in get_center_of_mass. Incorrect number of returned values from CoppeliaSim. (Expected: 3)");
    }
    DQ center_of_mass = data.output_floats[0]*i_ + data.output_floats[1]*j_
            + data.output_floats[2]*k_;
    return center_of_mass;
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
DQ DQ_VrepInterface::get_center_of_mass(const std::string& link_name, const REFERENCE_FRAMES& reference_frame, const std::string& function_name, const std::string& obj_name)
{    
    return get_center_of_mass(_get_handle_from_map(link_name), reference_frame,function_name, obj_name);
}


/**
 * @brief This method returns the mass of an object on the CoppeliaSim scene.
 * @param handle The handle of the object from which we want to extract the mass.
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
 *              int handle = vi.get_object_handle(link);
 *              double mass = vi.get_mass(handle);
 *                 // or    = vi.get_mass(handle, "get_mass")
 *                 // or    = vi.get_mass(handle, "get_mass","DQRoboticsApiCommandServer")
 *
 */
double DQ_VrepInterface::get_mass(const int& handle, const std::string& function_name, const std::string& obj_name)
{
    int outFloatCnt;
    float* output_floats;
    int return_code = _call_script_function(function_name, obj_name, {handle}, {}, {},
                                            0, nullptr, &outFloatCnt, &output_floats,0, nullptr);
    if (return_code != 0)
    {std::cout<<"Remote function call failed. Error: "<<return_code<<std::endl;}
    call_script_data data = _extract_call_script_data_from_pointers(return_code, 0, nullptr, outFloatCnt, output_floats, 0, nullptr);
    if (data.output_floats.size() != 1){
        throw std::range_error("Error in get_mass. Incorrect number of returned values from CoppeliaSim. (Expected: 1)");
    }
    return data.output_floats[0];
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
    return get_mass(_get_handle_from_map(link_name), function_name,obj_name);
}

/**
* @brief This protected method calls remotely a CoppeliaSim script function.
* @param function_name The name of the script function to call in the specified script.
* @param obj_name The name of the object where the script is attached to.
* @param input_ints The input integer values.
* @param input_floats The input float values.
* @param input_strings The input string values.
* @param outIntCnt (pointer) The number of returned integer values.
* @param output_ints (pointer) The returned integer values.
* @param outFloatCnt (pointer) The number of returned floating-point values.
* @param output_floats (pointer) The returned floating-point values.
* @param outStringCnt (pointer)  The number of returned strings.
* @param output_strings (pointer) The returned strings.
* @param scripttype The type of the script. (Default: ST_CHILD)
* @param opmode The operation mode. (Default: OP_BLOCKING)
* @returns The return code of the simxCallScriptFunction method.
*
*    Example:
*        int outIntCnt;
*        int* output_ints;
*        int outFloatCnt;
*        float* output_floats;
*        int outStringCnt;
*        char* output_strings;
*        int return_code = _call_script_function(function_name, obj_name, {}, {}, {},&outIntCnt, &output_ints, &outFloatCnt, &output_floats, &outStringCnt, &output_strings);
*
*        //To get the returned values, you can use _extract_call_script_data_from_pointers().
*        //Example:
*         call_script_data data = _extract_call_script_data_from_pointers(return_code, outIntCnt, output_ints, outFloatCnt, output_floats, outStringCnt, output_strings);
*
*
*/
int DQ_VrepInterface::_call_script_function(const std::string&  function_name, const std::string&  obj_name, const std::vector<int>& input_ints, const std::vector<float>& input_floats, const std::vector<std::string> &input_strings,
                                            int* outIntCnt, int** output_ints, int* outFloatCnt, float** output_floats, int* outStringCnt, char** output_strings,
                                            const SCRIPT_TYPES& scripttype, const OP_MODES& opmode)
{
    const int stringsize = input_strings.size();
    std::string one_string;
    if (stringsize >0)
    {
        for(int i = 0; i < stringsize; ++i)
        {
            one_string += input_strings[i]+'\0';
        }
    }

    int return_code = simxCallScriptFunction(clientid_, obj_name.c_str(), _remap_script_type(scripttype), function_name.c_str(),
                                             input_ints.size(), input_ints.data(), input_floats.size(), input_floats.data(), stringsize, one_string.data(),
                                             0, nullptr, outIntCnt, output_ints, outFloatCnt, output_floats,  outStringCnt,
                                             output_strings, nullptr, nullptr, _remap_op_mode(opmode));
    return return_code;
}

