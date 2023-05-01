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
- Murilo M. Marinho        (murilomarinho@ieee.org)
*/

#include<dqrobotics/interfaces/vrep/DQ_VrepInterfaceMapElement.h>
namespace DQ_robotics
{
DQ_VrepInterfaceMapElement::DQ_VrepInterfaceMapElement(const int& handle)
{
    handle_ = handle;
}

bool DQ_VrepInterfaceMapElement::state_from_function_signature(const std::string &function_signature)
{
    if(set_states_map_.count(function_signature)==1)
        set_states_map_.at(function_signature) = true;
    else
    {
        set_states_map_.insert(std::pair<std::string,bool>(function_signature,false));
    }
    return set_states_map_.at(function_signature);
}

int DQ_VrepInterfaceMapElement::get_handle()
{
    return handle_;
}

}
