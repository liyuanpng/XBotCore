/*
   Copyright (C) 2016 Italian Institute of Technology

   Developer:
       Luca Muratore (2016-, luca.muratore@iit.it)
       
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

/**
 *
 * @author Luca Muratore (2016-, luca.muratore@iit.it)
*/

#include <XBotPlugin/RobotInterfaceXBotRT_test.h>

XBot::RobotInterfaceXBotRT_test::RobotInterfaceXBotRT_test( std::string name, // TBD pass a config file and inside put the name
                                                        std::shared_ptr<XBot::IXBotModel> model, 
                                                        std::shared_ptr<XBot::IXBotChain> chain,
                                                        std::shared_ptr<XBot::IXBotRobot> robot,
                                                        std::shared_ptr<XBot::IXBotFT> ft) :
                                                        XBotPlugin(name, model, chain, robot, ft)
{

}

bool XBot::RobotInterfaceXBotRT_test::init(void)
{
//     _robot_interface = XBot::RobotInterface::getRobot(name, 0, nullptr);
    return true;
}

void XBot::RobotInterfaceXBotRT_test::run(void)
{
    // read link pos
//     robot->get_robot_link_pos(_link_pos);
//     for( auto& temp: _link_pos) {
//             DPRINTF("link_pos: JOINT %d -> %f \n", temp.first, temp.second);
//     }
}

bool XBot::RobotInterfaceXBotRT_test::close(void)
{
    return true;
}

XBot::RobotInterfaceXBotRT_test::~RobotInterfaceXBotRT_test()
{
}

