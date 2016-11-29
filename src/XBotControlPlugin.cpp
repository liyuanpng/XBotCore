/*
 * Copyright (C) 2016 IIT-ADVR
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

#include <XCM/XBotControlPlugin.h>

namespace XBot {
    
XBotControlPlugin::XBotControlPlugin()
{

}

XBotControlPlugin::~XBotControlPlugin()
{

}

bool XBotControlPlugin::init(std::string path_to_config_file, 
                             std::string name, 
                             std::shared_ptr< IXBotModel > model, 
                             std::shared_ptr< IXBotChain > chain, 
                             std::shared_ptr< IXBotRobot > robot, 
                             std::shared_ptr< IXBotFT > ft)
{
    this->name = name;
    this->model = model;
    this->chain = chain;
    this->ft = ft;
    
    AnyMapPtr any_map = std::make_shared<AnyMap>();
    (*any_map)["XBotRobot"] = boost::any(robot);
    
    RobotInterface::Ptr robotinterface = RobotInterface::getRobot(path_to_config_file, any_map);
    
    return init_control_plugin(robotinterface);

}

void XBotControlPlugin::run(double time, double period)
{
    control_loop(time, period);
}


    
}