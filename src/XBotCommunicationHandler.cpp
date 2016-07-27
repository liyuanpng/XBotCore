#include <XBotCore/XBotCommunicationHandler.h>

#include <sys/stat.h>
#include <fcntl.h>

XBot::XBotCommunicationHandler::XBotCommunicationHandler(std::string config_file)
{
    std::ifstream fin(config_file);
    if ( fin.fail() ) {
        DPRINTF("Can not open %s\n", config_file.c_str());
    }

    const YAML::Node& x_bot_core_config = YAML::LoadFile(config_file)["x_bot_core"];
    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    joint_map_config = x_bot_core_config["joint_map_config"].as<std::string>();
    
    // initialize the model
    if( !model.init( urdf_path, srdf_path, joint_map_config ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
    }
    
    // generate the robot
    model.generate_robot();
    // get the robot
    robot = model.get_robot();
    // get the ft
    ft = model.get_ft_sensors();

    // motors
    for(auto& c : robot) {
        for(int i=0; i< c.second.size(); i++) {
            
            // initialize all the fd reading for the motors
            int actual_fd = open((std::string("/proc/xenomai/registry/rtipc/xddp/") + std::string("Motor_id_") + std::to_string(c.second[i])).c_str(), O_RDONLY);
            if(actual_fd > 0) {
                fd_read[c.second[i]] = actual_fd;
            }
            
            // initialize all the fd writing for the motors
            actual_fd = open((std::string("/proc/xenomai/registry/rtipc/xddp/") + std::string("rt_in_Motor_id_") + std::to_string(c.second[i])).c_str(), O_WRONLY);
            if(actual_fd > 0) {
                fd_write[c.second[i]] = actual_fd;
            }
            
            // initialize all the fd SDO reading for the motors
            actual_fd = open((std::string("/proc/xenomai/registry/rtipc/xddp/") + std::string("sdo_Motor_id_") + std::to_string(c.second[i])).c_str(), O_RDONLY);
            if(actual_fd > 0) {
                fd_sdo_read[c.second[i]] = actual_fd;
            }
            
            // initialize the mutex
            mutex[c.second[i]] = std::make_shared<std::mutex>();
            
            // initialize the pdo_motor
            if(fd_read.count(c.second[i])) {
                iit::ecat::advr::McEscPdoTypes actual_pdo_motor;
                int n_bytes = read(fd_read[c.second[i]], (void*)&actual_pdo_motor, sizeof(actual_pdo_motor));
                if(n_bytes > 0) {
                    pdo_motor[c.second[i]] = std::make_shared<iit::ecat::advr::McEscPdoTypes>(actual_pdo_motor);
                }   
            }

            // initialize the sdo info
            if(fd_sdo_read.count(c.second[i])) {
                XBot::sdo_info actual_sdo;
                n_bytes = read(fd_sdo_read.at(c.second[i]), (void*)&actual_sdo, sizeof(actual_sdo));
                if(n_bytes > 0) {
                    sdo_info[c.second[i]] = std::make_shared<XBot::sdo_info>(actual_sdo);
                }
            }
        }
    }
    
    //ft
    for(auto& ft_j : ft) {
        // initialize all the fd reading for the ft
        int actual_fd = open((std::string("/proc/xenomai/registry/rtipc/xddp/") + std::string("Ft_id_") + std::to_string(ft_j.second)).c_str(), O_RDONLY);
        if(actual_fd > 0) {
            fd_ft_read[ft_j.second] = actual_fd;
        }
        // initialize the mutex
        mutex[ft_j.second] = std::make_shared<std::mutex>();
    }
    

    
    // set thread name
//     name = "communication_handler"; //TBD understand why pthread_setname_np return code error 3
    // set thread period - not periodic
    task_period_t t;
    memset(&t, 0, sizeof(t));
    t.period = {0,1};
    period.task_time = t.task_time;
    period.period = t.period;
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

std::map< std::string, std::vector< int > > XBot::XBotCommunicationHandler::get_robot_map()
{
    return model.get_robot();
}

std::map< std::string, int > XBot::XBotCommunicationHandler::get_ft_sensors_map()
{
    return model.get_ft_sensors();
}


XBot::XBotCoreModel XBot::XBotCommunicationHandler::get_robot_model()
{
    return model;
}



void XBot::XBotCommunicationHandler::th_init(void *)
{
   DPRINTF("XBotCommunicationHandler INIT\n");
   std::fflush(stdout);
}

void XBot::XBotCommunicationHandler::th_loop(void *)
{  
    for( auto& f: fd_write) {
        mutex.at(f.first)->lock();
        iit::ecat::advr::McEscPdoTypes::pdo_tx actual_pdo_tx = pdo_motor.at(f.first)->pdo_data_tx;
        n_bytes = write(f.second, (void*)&(actual_pdo_tx), sizeof(actual_pdo_tx));
        
        // NOTE the single joint element can oly be controlled by either the RT or the N-RT so it should be commented!
        iit::ecat::advr::McEscPdoTypes actual_pdo_motor;  
        n_bytes = read(fd_read.at(f.first), (void*)&actual_pdo_motor, sizeof(actual_pdo_motor));
        (*pdo_motor.at(f.first)).pdo_data_rx = actual_pdo_motor.pdo_data_rx;
        
        mutex.at(f.first)->unlock();
    }

}

bool XBot::XBotCommunicationHandler::get_min_pos(int joint_id, float& min_pos)
{
    if( sdo_info.count(joint_id) ){
        min_pos = sdo_info.at(joint_id)->min_pos;
        return true;
    }
    min_pos = 0.0; // NOTE avoid to break the robot
    return false;
}

bool XBot::XBotCommunicationHandler::get_max_pos(int joint_id, float& max_pos)
{
    if( sdo_info.count(joint_id) ){
        max_pos = sdo_info.at(joint_id)->max_pos;
        return true;
    }
    max_pos = 0.0; // NOTE avoid to break the robot
    return false;
}

bool XBot::XBotCommunicationHandler::get_ctrl_status_cmd(int joint_id, uint16_t& ctrl_status_cmd)
{
    if( sdo_info.count(joint_id) ){
        ctrl_status_cmd = sdo_info.at(joint_id)->ctrl_status_cmd;
        return true;
    }
    return false;
}



bool XBot::XBotCommunicationHandler::get_link_pos(int joint_id, float& link_pos)
{
    mutex.at(joint_id)->lock();
    link_pos = pdo_motor.at(joint_id)->pdo_data_rx.link_pos;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_motor_pos(int joint_id, float& motor_pos)
{
    mutex.at(joint_id)->lock();
    motor_pos = pdo_motor.at(joint_id)->pdo_data_rx.motor_pos;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_link_vel(int joint_id, float& link_vel)
{
    mutex.at(joint_id)->lock();
    link_vel = pdo_motor.at(joint_id)->pdo_data_rx.link_vel;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_motor_vel(int joint_id, int16_t& motor_vel)
{
    mutex.at(joint_id)->lock();
    motor_vel = pdo_motor.at(joint_id)->pdo_data_rx.motor_vel;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_torque(int joint_id, int16_t& torque)
{
    mutex.at(joint_id)->lock();
    torque = pdo_motor.at(joint_id)->pdo_data_rx.torque;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_max_temperature(int joint_id, uint16_t& max_temperature)
{
    mutex.at(joint_id)->lock();
    max_temperature = pdo_motor.at(joint_id)->pdo_data_rx.max_temperature;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_fault(int joint_id, uint16_t& fault)
{
    mutex.at(joint_id)->lock();
    fault = pdo_motor.at(joint_id)->pdo_data_rx.fault;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_rtt(int joint_id, uint16_t& rtt)
{
    mutex.at(joint_id)->lock();
    rtt = pdo_motor.at(joint_id)->pdo_data_rx.rtt;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_op_idx_ack(int joint_id, uint16_t& op_idx_ack)
{
    mutex.at(joint_id)->lock();
    op_idx_ack = pdo_motor.at(joint_id)->pdo_data_rx.op_idx_ack;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_aux(int joint_id, float& aux)
{
    mutex.at(joint_id)->lock();
    aux = pdo_motor.at(joint_id)->pdo_data_rx.aux;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_pos_ref(int joint_id, const float& pos_ref)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.pos_ref = pos_ref;
    //DPRINTF("joint : %d - set pos ref : %f\n", joint_id, pdo_motor.at(joint_id)->pdo_data_tx.pos_ref);
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_vel_ref(int joint_id, const int16_t& vel_ref)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.vel_ref = vel_ref;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_tor_ref(int joint_id, const int16_t& tor_ref)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.tor_ref = tor_ref;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_gains(int joint_id, const std::vector< uint16_t >& gains)
{
    mutex.at(joint_id)->lock();
    for(int i = 0; i < gains.size(); i++) {
        pdo_motor.at(joint_id)->pdo_data_tx.gains[i] = gains[i];
    }
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_fault_ack(int joint_id, const int16_t& fault_ack)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.fault_ack = fault_ack;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_ts(int joint_id, const uint16_t& ts)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.ts = ts;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.op_idx_aux = op_idx_aux;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_aux(int joint_id, const float& aux)
{
    mutex.at(joint_id)->lock();
    pdo_motor.at(joint_id)->pdo_data_tx.aux = aux;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_ft(int ft_id, std::vector< float >& ft, int channels)
{
    mutex.at(ft_id)->lock();
    
    iit::ecat::advr::Ft6EscPdoTypes::pdo_rx actual_pdo_rx_ft;  
    n_bytes = read(fd_ft_read.at(ft_id), (void*)&actual_pdo_rx_ft, sizeof(actual_pdo_rx_ft));
    ft.resize(channels);
    std::memcpy(ft.data(), &(actual_pdo_rx_ft.force_X), channels*sizeof(float));
    
    mutex.at(ft_id)->unlock();
    return true;  
}

bool XBot::XBotCommunicationHandler::get_ft_fault(int ft_id, uint16_t& fault)
{
    mutex.at(ft_id)->lock();
    iit::ecat::advr::Ft6EscPdoTypes::pdo_rx actual_pdo_rx_ft;   
    n_bytes = read(fd_ft_read.at(ft_id), (void*)&actual_pdo_rx_ft, sizeof(actual_pdo_rx_ft));
    fault = actual_pdo_rx_ft.fault;
    
    mutex.at(ft_id)->unlock();
    return true;  
}

bool XBot::XBotCommunicationHandler::get_ft_rtt(int ft_id, uint16_t& rtt)
{
    mutex.at(ft_id)->lock();
    iit::ecat::advr::Ft6EscPdoTypes::pdo_rx actual_pdo_rx_ft;    
    n_bytes = read(fd_ft_read.at(ft_id), (void*)&actual_pdo_rx_ft, sizeof(actual_pdo_rx_ft));
    rtt = actual_pdo_rx_ft.rtt;
    mutex.at(ft_id)->unlock();
    return true;  
}



XBot::XBotCommunicationHandler::~XBotCommunicationHandler()
{
    printf("~XBotCommunicationHandler()\n");
}
