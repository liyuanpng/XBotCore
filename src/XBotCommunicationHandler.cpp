#include <XBotCore/XBotCommunicationHandler.h>

#include <sys/stat.h>
#include <fcntl.h>

XBot::XBotCommunicationHandler::XBotCommunicationHandler(std::string config_file)
{
    std::ifstream fin(config_file);
    if ( fin.fail() ) {
        DPRINTF("Can not open %s\n", config_file.c_str());
//         return false;
    }

    const YAML::Node& x_bot_core_config = YAML::LoadFile(config_file)["x_bot_core"];
    urdf_path = x_bot_core_config["urdf_path"].as<std::string>();
    srdf_path = x_bot_core_config["srdf_path"].as<std::string>();
    joint_map_config = x_bot_core_config["joint_map_config"].as<std::string>();
    
    // initialize the model
    if( !model.init( urdf_path, srdf_path, joint_map_config ) ) {
        DPRINTF("ERROR: model initialization failed, please check the urdf_path and srdf_path in your YAML config file.\n");
//         return false;
    }
    
    // generate the robot
    model.generate_robot();
    // get the robot
    robot = model.get_robot();
    
    
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
            
            // initialize the mutex
            mutex[c.second[i]] = std::make_shared<std::mutex>();
            
            // initialize the pdo
            iit::ecat::advr::McEscPdoTypes actual_pdo;
            int n_bytes = read(fd_read[c.second[i]], (void*)&actual_pdo, sizeof(actual_pdo));
            pdo[c.second[i]] = std::make_shared<iit::ecat::advr::McEscPdoTypes>(actual_pdo);
        }
    }
    
    

    
    // set thread name
//     name = "c"; TBD understand why pthread_setname_np return code error 3
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


void XBot::XBotCommunicationHandler::th_init(void *)
{
   DPRINTF("INIT\n");
   std::fflush(stdout);
}

void XBot::XBotCommunicationHandler::th_loop(void *)
{
    
    for( auto& f: fd_write) {
        mutex.at(f.first)->lock();
        iit::ecat::advr::McEscPdoTypes::pdo_tx actual_pdo_tx = pdo.at(f.first)->pdo_data_tx;
        n_bytes = write(f.second, (void*)&(actual_pdo_tx), sizeof(actual_pdo_tx));
        
        // NOTE the single joint element can oly be controlled by either the RT or the N-RT
//         iit::ecat::advr::McEscPdoTypes actual_pdo;  
//         n_bytes = read(fd_read.at(f.first), (void*)&actual_pdo, sizeof(actual_pdo));
//         *pdo[f.first] = actual_pdo;
        
        mutex.at(f.first)->unlock();
    }



}

bool XBot::XBotCommunicationHandler::get_link_pos(int joint_id, float& link_pos)
{
    mutex.at(joint_id)->lock();
    link_pos = pdo.at(joint_id)->pdo_data_rx.link_pos;
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::get_motor_pos(int joint_id, float& motor_pos)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    motor_pos = pdo.at(joint_id)->pdo_data_rx.motor_pos;
    return true;
}

bool XBot::XBotCommunicationHandler::get_link_vel(int joint_id, float& link_vel)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    link_vel = pdo.at(joint_id)->pdo_data_rx.link_vel;
    return true;
}

bool XBot::XBotCommunicationHandler::get_motor_vel(int joint_id, int16_t& motor_vel)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    motor_vel = pdo.at(joint_id)->pdo_data_rx.motor_vel;
    return true;
}

bool XBot::XBotCommunicationHandler::get_torque(int joint_id, int16_t& torque)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    torque = pdo.at(joint_id)->pdo_data_rx.torque;
    return true;
}

bool XBot::XBotCommunicationHandler::get_max_temperature(int joint_id, uint16_t& max_temperature)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    max_temperature = pdo.at(joint_id)->pdo_data_rx.max_temperature;
    return true;
}

bool XBot::XBotCommunicationHandler::get_fault(int joint_id, uint16_t& fault)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    fault = pdo.at(joint_id)->pdo_data_rx.fault;
    return true;
}

bool XBot::XBotCommunicationHandler::get_rtt(int joint_id, uint16_t& rtt)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    rtt = pdo.at(joint_id)->pdo_data_rx.rtt;
    return true;
}

bool XBot::XBotCommunicationHandler::get_op_idx_ack(int joint_id, uint16_t& op_idx_ack)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    op_idx_ack = pdo.at(joint_id)->pdo_data_rx.op_idx_ack;
    return true;
}

bool XBot::XBotCommunicationHandler::get_aux(int joint_id, float& aux)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    aux = pdo.at(joint_id)->pdo_data_rx.aux;
    return true;
}

bool XBot::XBotCommunicationHandler::set_pos_ref(int joint_id, const float& pos_ref)
{
    mutex.at(joint_id)->lock();
    pdo.at(joint_id)->pdo_data_tx.pos_ref = pos_ref;
    DPRINTF("set pos ref : %f\n", pdo.at(joint_id)->pdo_data_tx.pos_ref);
    mutex.at(joint_id)->unlock();
    return true;
}

bool XBot::XBotCommunicationHandler::set_vel_ref(int joint_id, const int16_t& vel_ref)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    pdo.at(joint_id)->pdo_data_tx.vel_ref = vel_ref;
    return true;
}

bool XBot::XBotCommunicationHandler::set_tor_ref(int joint_id, const int16_t& tor_ref)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    pdo.at(joint_id)->pdo_data_tx.tor_ref = tor_ref;
    return true;
}

bool XBot::XBotCommunicationHandler::set_gains(int joint_id, const std::vector< uint16_t >& gains)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    for(int i = 0; i < gains.size(); i++) {
            pdo.at(joint_id)->pdo_data_tx.gains[i] = gains[i];
    }
    return true;
}

bool XBot::XBotCommunicationHandler::set_fault_ack(int joint_id, const int16_t& fault_ack)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    pdo.at(joint_id)->pdo_data_tx.fault_ack = fault_ack;
    return true;
}

bool XBot::XBotCommunicationHandler::set_ts(int joint_id, const uint16_t& ts)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    pdo.at(joint_id)->pdo_data_tx.ts = ts;
    return true;
}

bool XBot::XBotCommunicationHandler::set_op_idx_aux(int joint_id, const uint16_t& op_idx_aux)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    pdo.at(joint_id)->pdo_data_tx.op_idx_aux = op_idx_aux;
    return true;
}

bool XBot::XBotCommunicationHandler::set_aux(int joint_id, const float& aux)
{
    std::lock_guard<std::mutex> lock(*mutex.at(joint_id));
    pdo.at(joint_id)->pdo_data_tx.aux = aux;
    return true;
}



XBot::XBotCommunicationHandler::~XBotCommunicationHandler()
{

}
