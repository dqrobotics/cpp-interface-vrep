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

#ifndef DQ_ROBOTICS_INTERFACE_MAP_ELEMENT_GUARD
#define DQ_ROBOTICS_INTERFACE_MAP_ELEMENT_GUARD
#include<map>
#include<string>
namespace DQ_robotics
{
class DQ_VrepInterfaceMapElement
{
private:
    std::map<std::string,double> set_states_map_;
    int handle_;
public:
    explicit DQ_VrepInterfaceMapElement(const int& handle);
    bool state_from_function_signature(const std::string& function_signature);
    int get_handle();
};

}
#endif
