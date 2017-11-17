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

#ifndef __XCM_EXAMPLES_HOMING_EXAMPLE_H__
#define __XCM_EXAMPLES_HOMING_EXAMPLE_H__

#include <XCM/XBotControlPlugin.h>

namespace XBot {

    class HomingExample : public XBotControlPlugin {

    public:

        HomingExample();

        virtual ~HomingExample(){ Logger::info() << "~HomingExample()" << Logger::endl(); }
        
        virtual bool init_control_plugin(XBot::Handle::Ptr handle);

        virtual bool close();

        virtual void on_start(double time);

        virtual void on_stop(double time);

    protected:


        virtual void control_loop(double time, double period);

    private:
        
        XBot::SharedObject<XBot::JointIdMap> _pos_ref_map;

        RobotInterface::Ptr _robot;
        Eigen::VectorXd _q0, _q_home, _q, _k, _d, _k0, _d0, _qref;
        double _time, _homing_time, _first_loop_time;

        ForceTorqueSensor::ConstPtr _l_arm_ft;
        Eigen::Matrix<double, 6, 1> _l_arm_wrench;

        double _l_hand_pos;
        double _l_hand_ref;
        bool _close_hand;

    };

}

#endif // __XCM_EXAMPLES_HOMING_EXAMPLE_H__
