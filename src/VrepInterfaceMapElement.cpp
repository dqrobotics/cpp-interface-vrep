#include<dqrobotics/interfaces/VrepInterfaceMapElement.h>
namespace DQ_robotics
{
VrepInterfaceMapElement::VrepInterfaceMapElement(const int& handle)
{
    handle_ = handle;
}

bool VrepInterfaceMapElement::state_from_function_signature(const std::string &function_signature)
{
    if(set_states_map_.count(function_signature)==1)
        set_states_map_.at(function_signature) = true;
    else
    {
        set_states_map_.insert(std::pair<std::string,bool>(function_signature,false));
    }
    return set_states_map_.at(function_signature);
}

int VrepInterfaceMapElement::get_handle()
{
    return handle_;
}

}
