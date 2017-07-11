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
                             std::shared_ptr<PluginStatus> cstatus,
                             XBot::SharedMemory::Ptr shared_memory,
                             std::shared_ptr< XBot::IXBotJoint > joint,
                             std::shared_ptr< XBot::IXBotModel > model,
                             std::shared_ptr< XBot::IXBotFT > ft,
                             std::shared_ptr< XBot::IXBotIMU > imu,
                             std::shared_ptr< XBot::IXBotHand> hand  )
{
    this->name = name;
    this->set_xbotcore_joint(joint);
    this->set_xbotcore_model(model);
    this->set_xbotcore_ft(ft);
    this->set_xbotcore_imu(imu);
    this->set_xbotcore_hand(hand);

    AnyMapPtr any_map = std::make_shared<AnyMap>();
    (*any_map)["XBotJoint"] = boost::any(joint);
    (*any_map)["XBotFT"] = boost::any(ft);
    (*any_map)["XBotIMU"] = boost::any(imu);
    (*any_map)["XBotHand"] = boost::any(hand);
    
    RobotInterface::Ptr robotinterface = RobotInterface::getRobot(path_to_config_file, any_map);
    
    // initialize the command port
    command.init(name + "_cmd");
    _custom_status = cstatus;

    return init_control_plugin(path_to_config_file, shared_memory, robotinterface);

}

void XBotControlPlugin::run(double time, double period)
{
    
    control_loop(time, period);
}

}
