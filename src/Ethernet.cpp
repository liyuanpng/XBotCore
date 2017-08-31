/*
   Boards_ctrl_basic.cpp

   Copyright (C) 2012 Italian Institute of Technology

   Developer:
       Alessio Margan (2013-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2013-, alessio.margan@iit.it)
*/


#include <string.h>

#include <Ethernet.h>
#include <Boards_exception.h>

static const std::vector<float> homeVel(25,25);

static const std::vector<float> homePos = {
    // lower body #15
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
//  1,  2,  3,  4,  5,  6,  7,  8,  9, 10, 11, 12, 13, 14, 15
    // upper body #10
    0, 87,  0, -3,  0,-87,  0, -3,  0,  0};
// 16, 17, 18, 19, 20, 21, 22, 23, 24, 25


// boards ID
std::vector<int> r_leg = {  4,  6,  7,  8,  9, 10};
std::vector<int> l_leg = {  5, 11, 12, 13, 14, 15};
std::vector<int> waist = {  1,  2, 3};
std::vector<int> r_arm = { 16, 17, 18 ,19};
std::vector<int> l_arm = { 20, 21, 22, 23};
std::vector<int> neck  = {}; //{ 24, 25};


Ethernet::Ethernet(const char * config): Boards_ctrl_ext(config) {

    name = "boards_ctrl_basic";
    period.period = {0,1000};

#ifdef __XENO__
    schedpolicy = SCHED_FIFO;
#else
    schedpolicy = SCHED_OTHER;
#endif
    priority = sched_get_priority_max(schedpolicy);
    stacksize = PTHREAD_STACK_MIN;

    xddp_test = new Write_XDDP_pipe(std::string(name), 4096);

}

Ethernet::~Ethernet() {

    delete xddp_test;

    std::cout << "~" << typeid(this).name() << std::endl;
}


void Ethernet::init_internal() {

    // configure dsp and start bc data
    // NOT start motor controller
    init();
    // read current position and set as homing
    homing();
    test();
    trj_flag = 0;

}

void Ethernet::init(){
    
    init_internal();
    
}

int Ethernet::recv_from_slave(){
    
     uint8_t cmd;
    static char user_data[1024];

    try {

        sense();
        
        user_input(cmd);
        return 1;
        
        } catch ( boards_error &e ) {
        DPRINTF("FATAL ERROR in %s ... %s\n", __FUNCTION__, e.what());
        // handle error .... exit thread
        // exit {rt,nrt}_periodic_thread function
        _run_loop = 0;

    } catch ( boards_warn &e ) {
        DPRINTF("WARNING in %s ... %s\n", __FUNCTION__, e.what());
        // handle warning
    }
     return 0;
}

int Ethernet::send_to_slave(){
    
    try {          
       
            sprintf(user_data, "%ld\n", get_time_ns());
            xddp_test->write((void*)user_data, strlen(user_data));
        
            return 1;
        } catch ( boards_error &e ) {
        DPRINTF("FATAL ERROR in %s ... %s\n", __FUNCTION__, e.what());
        // handle error .... exit thread
        // exit {rt,nrt}_periodic_thread function
        _run_loop = 0;

        } catch ( boards_warn &e ) {
            DPRINTF("WARNING in %s ... %s\n", __FUNCTION__, e.what());
            // handle warning
        }
    return 0;
}

// int Ethernet::user_loop(void) {
// 
//     int bId;
//     static double freq_Hz = 1;
//     if ( g_tStart <= 0 ) {
//         g_tStart = get_time_ns();
//     }
//     uint64_t dt_ns = get_time_ns() - g_tStart;
//     double  trj = sin((2.0 * M_PI * freq_Hz * dt_ns)/1e9); // -1 .. 1
//     if ( trj_flag == 1 ) {
//         for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
//             bId = it->first;
//             _pos[bId-1] = _home[bId-1] + (DEG2mRAD(10) * trj);
//         }
//         //
//         move(MV_POS|MV_VEL|MV_TOR|MV_STF);
//     }
// 
//     return 0;
// }

int Ethernet::user_input(uint8_t &cmd) {

    static int toggle = 1;
    McBoard * b;

    // fakes ids ... used for test group functions
    uint8_t bIds[2] = { 88, 99};
    int pos[2];
    short vel[2];

    int nbytes = Boards_ctrl_ext::user_input((void*)&cmd, sizeof(cmd));

    if ( nbytes <= 0 ) {
        return nbytes;
    }

    switch ( cmd ) {
        case 'S':
            DPRINTF("Start control ...r_arm\n");
            start_stop_set_control(r_arm,true);
            DPRINTF("Start control ...r_leg\n");
            start_stop_set_control(r_leg,true);
            DPRINTF("Start control ...l_arm\n");
            start_stop_set_control(l_arm,true);
            DPRINTF("Start control ...l_leg\n");
            start_stop_set_control(l_leg,true);
            DPRINTF("Start control ...waist\n");
            start_stop_set_control(waist,true);
            break;
        case '1':
            //DPRINTF("Start control single \n");
            //start_stop_single_control(8,true);
            DPRINTF("Start control single \n");
            start_stop_single_control(15,true);
            break;
        case '2':
            DPRINTF("Start control ...l_arm\n");
            start_stop_set_control(l_arm,true);
            DPRINTF("Start control ...r_arm\n");
            start_stop_set_control(r_arm,true);
            break;
        case '3':
            DPRINTF("Start control ...l_leg\n");
            start_stop_set_control(l_leg,true);
            DPRINTF("Start control ...r_leg\n");
            start_stop_set_control(r_leg,true);
            break;
        case 'h':
            DPRINTF("Set home pos\n");
            homing(homePos, homeVel);
            //test();
            break;
        case 'A':
            DPRINTF("Set pos ref to median point of range pos\n");
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = (b->_max_pos + b->_min_pos) / 2;
            }
            move();
            break;
        case 'a':
            DPRINTF("Do something else\n");
            toggle *= -1;
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = _home[b->bId-1] + DEG2mRAD(5) * toggle;
            }
            move();
            break;
        case '[':
            DPRINTF("Do something ++++ \n");
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = _ts_bc_data[b->bId-1].raw_bc_data.mc_bc_data.Position + DEG2mRAD(0.5);
            }
            move();
            break;
        case ']':
            DPRINTF("Do something ----\n");
            for ( auto it = _mcs.begin(); it != _mcs.end(); it++ ) {
                b = it->second;
                _pos[b->bId-1] = _ts_bc_data[b->bId-1].raw_bc_data.mc_bc_data.Position - DEG2mRAD(0.5);
            }
            move();
            break;

        case 't':

            DPRINTF("trajectory\n");
            homing();
            g_tStart = get_time_ns();
            trj_flag = ! trj_flag;
            break;

        case 'X':
            pos_group.clear();
            pos_group[88] = 0x00DEAD00;
            pos_group[99] = 0x11BEEF11;
            set_position_group(pos_group);
            pos_vel_group.clear();
            pos_vel_group[88] = std::make_pair(0x00DEAD00, 0xCACA);
            pos_vel_group[99] = std::make_pair(0x11BEEF11, 0x7777);
            set_position_velocity_group(pos_vel_group);
            break;

        case 'x':
            pos[88] = 0x00DEAD00;
            pos[99] = 0x11BEEF11;
            set_position_group(bIds,pos,2);
            vel[88] = 0xCACA;
            vel[99] = 0x7777;
            set_position_velocity_group(bIds,pos,vel,2);
            break;

        case 'j':
            pos[88] = 0x00DEAD00;
            pos[99] = 0x11BEEF11;
            set_gravity_compensation(pos,sizeof(pos));
            break;

        case 'P':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 1000, 0, 0);
            break;
        case 'p':
            _mcs[19]->set_PID_increment(POSITION_GAINS, -1000, 0, 0);
            break;
        case 'I':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, 1, 0);
            break;
        case 'i':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, -1, 0);
            break;
        case 'D':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, 0, 50);
            break;
        case 'd':
            _mcs[19]->set_PID_increment(POSITION_GAINS, 0, 0, -50);
            break;


        case 'U':
            // works !!! need to handle dsp reboot ... exit app 
            _mcs[1]->setItem(CMD_UPGRADE, 0, 0);
            break;


        case '@':
            throw(boards_warn(std::string("Hi this is a boards warning")));
            // ... never break .....
            //break;

        case '#':
            throw(boards_error(std::string("Hi this is an boards error")));
            // ... never break .....
            //break;

        case '!':
            throw(std::runtime_error(std::string("Hi this is a not handled except")));
            // ... never break .....
            //break;

        default:
            DPRINTF("Stop control ...\n");
            start_stop_control(false);
            clear_mcs_faults();
            break;
    }

    return nbytes;
}

