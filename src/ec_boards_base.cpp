/*
   Copyright (C) Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)
       Giuseppe Rigano (2017, giuseppe.rigano@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
 * @author Giuseppe Rigano (2017-, giuseppe.rigano@iit.it)
*/

//#include <iit/advr/ec_boards_base.h>
#include <XBotCore/ec_boards_base.h>

Ec_Boards_base::Ec_Boards_base ( const char * config_yaml ) : Ec_Boards_ctrl(config_yaml) {
}

Ec_Boards_base::~Ec_Boards_base() {

    std::cout << "~" << typeid ( this ).name() << std::endl;
    stop_motors();
    iit::ecat::print_stat ( s_loop );
    
}


void Ec_Boards_base::init_internal () {

    const YAML::Node config = get_config_YAML_Node();

    // init Ec_Boards_ctrl
    if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
        throw "something wrong";
    }

    get_esc_map_byclass ( pows );
    DPRINTF ( "found %lu pows\n", pows.size() );
    get_esc_map_byclass ( powCmns );
    DPRINTF ( "found %lu powCmns\n", powCmns.size() );
    
    // walkman power/battery board turn on all ESCs
    if ( pows.size() == 1 && slaves.size() == 1 ) {
        
        while ( ! pows[1]->power_on_ok() ) {
            osal_usleep(1000000);
            pows[1]->readSDO_byname("status");
            pows[1]->handle_status();
        }
        // power on
        Ec_Boards_ctrl::shutdown(false);
        // wait boards boot up
        sleep(6);
        // init Ec_Boards_ctrl
        if ( Ec_Boards_ctrl::init() != iit::ecat::advr::EC_BOARD_OK ) {
            throw "something wrong";
        }

    }

    get_esc_map_byclass ( motors );
    DPRINTF ( "found %lu motors\n", motors.size() );
    get_esc_map_byclass ( fts );
    DPRINTF ( "found %lu fts\n", fts.size() );
    get_esc_map_byclass ( foot_sensors );
    DPRINTF ( "found %lu foot_sensors\n", foot_sensors.size() );
    get_esc_map_byclass ( imus );
    DPRINTF ( "found %lu imus\n", imus.size() );
    get_esc_map_byclass ( tests );
    DPRINTF ( "found %lu tests\n", tests.size() );

    for ( auto const& item : motors ) {
        DPRINTF ("pos %d == %d rid2Pos() rid %d ==  %d pos2Rid()\n",
                 item.first, rid2Pos(item.second->get_robot_id()),
                 item.second->get_robot_id(), pos2Rid(item.first) );
        assert( item.first == rid2Pos(item.second->get_robot_id()) && item.second->get_robot_id() == pos2Rid(item.first) );
    }

    
    init_preOP();

    if ( set_operative() <= 0 ) {
        throw "something else wrong";
    }

    start_time = iit::ecat::get_time_ns();
    tNow, tPre = start_time;

//     if ( config["ec_boards_base"]["create_pipes"].as<bool>() ) {
//         //xddps_init();
//     }

    init_OP();
}

void Ec_Boards_base::init(){
    
    init_internal();
    
    return;
}

int Ec_Boards_base::recv_from_slave(){
    
    tNow = iit::ecat::get_time_ns();
    s_loop ( tNow - tPre );
    tPre = tNow;

    try {

        if ( recv_from_slaves ( timing ) != iit::ecat::advr::EC_BOARD_OK ) {
            // TODO
            DPRINTF ( "recv_from_slaves FAIL !\n" );
           
        }
         } catch ( iit::ecat::EscWrpError &e ) {
        std::cout << e.what() << std::endl;
        return 1;
    }
     return 0;
}

int Ec_Boards_base::send_to_slave(){
    
    try {          
       
            send_to_slaves();
        
        } catch ( iit::ecat::EscWrpError &e ) {
            std::cout << e.what() << std::endl;
            return 1;
        }
    return 0;
}

void Ec_Boards_base::xddps_init ( void ) {

    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Ft6ESC * ft;
    XDDP_pipe * xddp;

    for ( auto const& item : motors ) {
        slave_pos = item.first;
        moto = item.second;
        xddp = new XDDP_pipe();
        xddp->init ( "Motor_id_"+std::to_string ( moto->get_robot_id() ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : fts ) {
        slave_pos = item.first;
        ft = item.second;
        xddp = new XDDP_pipe();
        xddp->init ( "Ft_id_"+std::to_string ( ft->get_robot_id() ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : pows ) {
        slave_pos = item.first;
        xddp = new XDDP_pipe();
        xddp->init ( "Pow_pos_"+std::to_string ( slave_pos ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : powCmns ) {
        slave_pos = item.first;
        xddp = new XDDP_pipe();
        xddp->init ( "PowCmn_pos_"+std::to_string ( slave_pos ) );
        xddps[slave_pos] = xddp;
    }

    for ( auto const& item : tests ) {
        slave_pos = item.first;
        xddp = new XDDP_pipe();
        xddp->init ( "Test_pos_"+std::to_string ( slave_pos ) );
        xddps[slave_pos] = xddp;
    }

}

void Ec_Boards_base::xddps_loop ( void ) {

    int 	slave_pos;
    uint16_t	esc_type;

    for ( auto const& item : xddps ) {

        slave_pos = item.first;
        esc_type = slaves[slave_pos]->get_ESC_type();
        switch ( esc_type ) {
        case iit::ecat::advr::LO_PWR_DC_MC :
        case iit::ecat::advr::HI_PWR_AC_MC :
        case iit::ecat::advr::HI_PWR_DC_MC :
            item.second->xddp_write ( motors[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::Motor::motor_pdo_rx_t, iit::ecat::advr::Motor>(slave_pos));
            break;
        case iit::ecat::advr::FT6 :
            item.second->xddp_write ( fts[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::Ft6ESC::pdo_rx_t, iit::ecat::advr::Ft6ESC>(slave_pos));
            break;
        case iit::ecat::advr::POW_BOARD :
            item.second->xddp_write ( pows[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::PowESC::pdo_rx_t, iit::ecat::advr::PowESC>(slave_pos));
            break;
        case iit::ecat::advr::POW_CMN_BOARD :
            item.second->xddp_write ( powCmns[slave_pos]->getRxPDO() );
            //item.second->xddp_write(getRxPDO<iit::ecat::advr::PowComanESC::pdo_rx_t, iit::ecat::advr::PowComanESC>(slave_pos));
            break;
        case iit::ecat::advr::EC_TEST :
            item.second->xddp_write ( tests[slave_pos]->getRxPDO() );
            //item.second->xddp_write ( getRxPDO<iit::ecat::advr::TestEscPdoTypes::pdo_rx,iit::ecat::advr::TestESC>(slave_pos) );
            break;

        default:
            DPRINTF ( "[WARN] ESC type %d NOT handled %s\n", esc_type, __PRETTY_FUNCTION__ );
            break;
        }
    }
}

/**
 * NOTE this is a step reference !!!
 * LoPowerMotor (i.e. Coman) has a trajectory generator with max speed 0.5 rad/s
 * HiPowerMotor does NOT have it
 */
bool Ec_Boards_base::go_there ( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
                                       const std::map<int,float> &target_pos,
                                       float eps, bool debug ) {

    int cond, cond_cnt, cond_sum;
    float pos_ref, motor_err, link_err, motor_link_err;
    int slave_pos;
    iit::ecat::advr::Motor * moto;
    iit::ecat::advr::Motor::motor_pdo_rx_t motor_pdo_rx;
    std::vector<int> truth_vect;

    cond = cond_cnt = cond_sum = 0;

    for ( auto const& item : motor_set ) {
        slave_pos = item.first;
        moto =  item.second;

        // check in the target_pos map if the current slave_pos exist
        try {
            pos_ref = target_pos.at ( slave_pos );
        } catch ( const std::out_of_range& oor ) {
            continue;
        }

        motor_pdo_rx = moto->getRxPDO();
        //getRxPDO(slave_pos, motor_pdo_rx);
        moto->set_posRef ( pos_ref );

        link_err = fabs ( motor_pdo_rx.link_pos  - pos_ref );
        motor_err = fabs ( motor_pdo_rx.motor_pos - pos_ref );
        motor_link_err = fabs ( motor_pdo_rx.motor_pos - motor_pdo_rx.link_pos );

        cond = ( link_err <= eps || motor_err <= eps ) ? 1 : 0;
        cond_cnt++;
        cond_sum += cond;

        if ( debug ) {
            truth_vect.push_back ( cond );
            if ( ! cond ) {
                DPRINTF ( "rId %d\tposRef %f \t link %f{%f} \t motor %f{%f} \t |motor-link|{%f}\n",
                          pos2Rid ( slave_pos ), pos_ref,
                          motor_pdo_rx.link_pos, link_err,
                          motor_pdo_rx.motor_pos, motor_err,
                          motor_link_err );
            }
        }
    }

    if ( debug ) {
        DPRINTF ( "---\n" );
        for ( auto b : truth_vect ) {
            DPRINTF ( "%d ",b );
        }
        DPRINTF ( "\n=^=\n" );
    }

    return ( cond_cnt == cond_sum );
}

void Ec_Boards_base::remove_rids_intersection(std::vector<int> &start_dest, const std::vector<int> &to_remove)
{
    start_dest.erase(
        std::remove_if(start_dest.begin(), start_dest.end(),
            [to_remove](const int &rid) {
                return ( std::find(to_remove.begin(),to_remove.end(),rid) != to_remove.end()); }), 
        start_dest.end()
    );
    
}


// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
