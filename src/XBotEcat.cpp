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

#include <XBotCore/XBotEcat.h>

#include <string.h>

XBot::XBotEcat::XBotEcat(const char* config_yaml) : Ec_Thread_Boards_base(config_yaml)
{
    // set thread name
    const YAML::Node& board_ctrl = root_cfg["x_bot_ecat"]; // TBD check that the Node is defined
    set_thread_name(board_ctrl["name"].as<std::string>()); // TBD check that name is defined
    
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,1};
    set_thread_period(t);
    
    // set thread priority
    set_thread_priority();
    
}

void XBot::XBotEcat::set_thread_name(std::string thread_name)
{
    // save the thread name
    this->thread_name = thread_name;
    // set thread name
    name = this->thread_name.c_str();
}

std::string XBot::XBotEcat::get_thread_name(void)
{
    return thread_name;
}

void XBot::XBotEcat::set_thread_period(task_period_t t)
{
    period.task_time = t.task_time;
    period.period = t.period;
}

void XBot::XBotEcat::set_thread_priority()
{

    // set scheduler policy
#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    
    // set scheduler priority and stacksize
    priority = sched_get_priority_max(schedpolicy);
    stacksize = 0; // not set stak size !!!! YOU COULD BECAME CRAZY !!!!!!!!!!!!
}

void XBot::XBotEcat::init_sdo_xddp()
{
    std::shared_ptr<XDDP_pipe> xddp;
    iit::ecat::advr::Motor * moto;
    
    for ( auto const& item : motors ) {
        moto = item.second;
        xddp = std::make_shared<XDDP_pipe>();
        xddp->init ( "sdo_Motor_id_"+std::to_string ( moto->get_robot_id() ) );
        sdo_xddps[item.first] = xddp;
    }
}

void XBot::XBotEcat::write_sdo_info()
{
    XBot::sdo_info sdo;
    for ( auto const& p : sdo_xddps ) {
        motors.at(p.first)->readSDO<float>("Min_pos", sdo.min_pos);
        motors.at(p.first)->readSDO<float>("Max_pos", sdo.max_pos);
        motors.at(p.first)->readSDO<uint16_t>("ctrl_status_cmd", sdo.ctrl_status_cmd);
        p.second->xddp_write<XBot::sdo_info>(sdo);
    }
}

void XBot::XBotEcat::init_preOP(void) 
{
    // TBD read from the config file how to start the motors
    iit::ecat::advr::Motor * moto;
    for ( auto const& m : motors ) {
        moto = m.second;
        if(moto->get_control_mode() == "pos_3b") {
            DPRINTF("Starting motor %d in pos_3b\n", moto->get_robot_id());
            moto->start(CTRL_SET_POS_MODE);
        }
        else if(moto->get_control_mode() == "mix_pos_3c") {
            DPRINTF("Starting motor %d in mix_pos_3c\n", moto->get_robot_id());
            moto->start(CTRL_SET_MIX_POS_MODE);
        }
        else {
            DPRINTF("Starting motor %d in idle\n", moto->get_robot_id());
            // NOTE idle or not reognized TBD check!
        }
//         if(moto->am_i_LpESC()) {
//             moto->start(CTRL_SET_POS_MODE);
//         }
//         else if(moto->am_i_HpESC()) {
//             moto->start(CTRL_SET_MIX_POS_MODE);
//         }
    }
    
    return;
}

void XBot::XBotEcat::init_OP(void)
{
    // open SDO XDDP
    init_sdo_xddp();
    // write SDO info
    write_sdo_info(); 
    // control init implemented by the derived classes
    control_init();
    return;
}

int XBot::XBotEcat::user_loop(void) {
    
    // call the control loop
    return control_loop();
}


XBot::XBotEcat::~XBotEcat() {
    printf("~XBotEcat()\n");
    
}
