/*
 * Copyright (C) 2017 IIT-ADVR
 * Author:
 * email:
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

#include <_MODULE_PREFIX__io_plugin.h>

/* Specify that the class XBotPlugin::_MODULE_PREFIX_ is a XBot RT plugin with name "_MODULE_PREFIX_" */
REGISTER_XBOT_IO_PLUGIN_(XBotPlugin::_MODULE_PREFIX_)

namespace XBotPlugin {

bool XBotPlugin::_MODULE_PREFIX_::init(std::string path_to_config_file)
{
    return true;
}

void XBotPlugin::_MODULE_PREFIX_::run()
{

}

void XBotPlugin::_MODULE_PREFIX_::close()
{

}



}