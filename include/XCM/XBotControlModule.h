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

#ifndef __XBOTCONTROLMODULE_XBOT_CONTROL_MODULE_H__
#define __XBOTCONTROLMODULE_XBOT_CONTROL_MODULE_H__


#include <XBotInterface/RobotInterface.h>
#include <XBotCore-interfaces/XBotPlugin.h>

namespace XBot {

    class XBotControlModule {

    public:

        typedef std::shared_ptr<XBotControlModule> Ptr;

        XBotControlModule();

        virtual std::string get_name() const = 0;

        virtual ~XBotControlModule();

        virtual bool init_control_module(std::string path_to_config_file,
                                         RobotInterface::Ptr robot) = 0;

        virtual void on_start(double time) = 0;

        virtual void on_stop(double time) = 0;

        void run(double time, double period);

        virtual void close() = 0;

    protected:

        virtual void control_loop(double time, double period) = 0;


    private:




    };

}

#endif
