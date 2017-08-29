/*
   Copyright (C) Italian Institute of Technology

   Developer:
       Alessio Margan (2015-, alessio.margan@iit.it)

*/

/**
 *
 * @author Alessio Margan (2015-, alessio.margan@iit.it)
*/

#ifndef __EC_THREAD_BOARDS_BASE_H__
#define __EC_THREAD_BOARDS_BASE_H__

// #include <iit/ecat/advr/ec_boards_iface.h>
// 
// #include <iit/advr/thread_util.h>
// #include <iit/ecat/advr/pipes.h>
// #include <iit/advr/trajectory.h>

#include <RobotControlInterface.h>
#include <queue>
#include <iostream>
#define ECAT_PTHREAD_STACK_SIZE (16*1024*1024) // 16MB

/**
 */


class Ec_Thread_Boards_base : public RobotControlInterface
// public iit::ecat::advr::Ec_Boards_ctrl 
{
public:

    virtual ~Ec_Thread_Boards_base();
    Ec_Thread_Boards_base ( const char * config_yaml );

//     virtual void th_init ( void * );
//     virtual void th_loop ( void * );

//     virtual int user_loop ( void ) = 0;
    
   virtual void init() {
       std::cout<<"init boardbase"<<std::endl;
       init_preOP();
       init_OP();
       
};
   virtual int recv_from_slave(){std::cout<<"recvfrom boardbase"<<std::endl;};
   virtual int send_to_slave(){std::cout<<"sendto boardbase"<<std::endl;};

protected :

    virtual void init_preOP ( void ) = 0;
    virtual void init_OP ( void ) = 0;

//     iit::ecat::stat_t  s_loop;
//     uint64_t start_time, tNow, tPre;
//     
//     void xddps_init ( void );
//     void xddps_loop ( void );
//     std::map<int,XDDP_pipe*> xddps;
// 
//     XDDP_pipe termInXddp;
//     XDDP_pipe debugOutXddp;
//     
//     std::map<int, iit::ecat::advr::Motor*>          motors;
//     std::map<int, iit::ecat::advr::Ft6ESC*>         fts;
//     std::map<int, iit::ecat::advr::FootSensorESC*>  foot_sensors;
//     std::map<int, iit::ecat::advr::ImuVnESC*>       imus;
//     std::map<int, iit::ecat::advr::PowESC*>         pows;
//     std::map<int, iit::ecat::advr::PowComanESC*>    powCmns;
//     std::map<int, iit::ecat::advr::TestESC*>        tests;
// 
//     void remove_rids_intersection(std::vector<int> &, const std::vector<int> &);
//     
//         
//     bool go_there( const std::map<int, iit::ecat::advr::Motor*> &motor_set,
//                                        const std::map<int,float> &target_pos,
//                                        float eps, bool debug );

private:

//     iit::ecat::ec_timing_t timing;
    
};




#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
