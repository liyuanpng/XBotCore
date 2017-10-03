/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __XCM_IPC_COMMON_MESSAGES_H__
#define __XCM_IPC_COMMON_MESSAGES_H__

#include <XBotCore-interfaces/XDomainCommunication.h>
#include <Eigen/Dense>

namespace XBot { namespace Messages {

/**
* @brief Class which acts as a std::string of maximum length equal to 40
* chars. It is commonly used to send textual commands over XDDP pipes.
*/
class ShortString {

public:

    ShortString(const std::string& string = "")
    {
        int str_end = std::min((int)string.length(), 40);
        int i;
        for( i = 0; i < str_end; i++ ){
            char_array[i] = string[i];
        }
        char_array[i] = '\0';
    }

    std::string str() const
    {
        return std::string(char_array);
    }

    ShortString& operator=(const std::string& string)
    {
        *this = ShortString(string);
        return *this;
    }

    ShortString& operator=(const char * string)
    {
        *this = ShortString(string);
        return *this;
    }

    operator std::string()
    {
        return str();
    }

private:

    char char_array[41];

};



struct TransformMessage {

    ShortString child_frame;
    ShortString parent_frame;
    Eigen::Affine3d pose;

};


}  }

#endif