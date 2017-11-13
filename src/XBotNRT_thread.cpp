#include <stdio.h>
#include <stdlib.h>

#include <XCM/XBotThread.h>
#include <XBotInterface/RtLog.hpp>

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void * XBot::nrt_thread ( Thread_hook_Ptr th_hook ) {
    timespec    t;

    // INIT
    th_hook->th_init ( 0 );

    Logger::info() << "THREAD INIT: name = " << th_hook->name << ", period " << th_hook->period.period.tv_usec << " us" << Logger::endl();


    int ret = 0;
#ifdef __XENO__
//     ret = pthread_set_name_np ( pthread_self(), ( *th_hook ).name );
#else
//     ret = pthread_setname_np ( pthread_self(), ( *th_hook ).name );
#endif
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }
    
    Logger::success(Logger::Severity::HIGH) << "Thread " << th_hook->name << ": start looping" << Logger::endl();

    clock_gettime ( CLOCK_MONOTONIC ,&t );

    while ( th_hook->_run_loop ) {

        clock_nanosleep ( CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL );

        // thread specific loop
        th_hook->th_loop ( 0 );

        // calculate next shot
        t.tv_nsec += ( th_hook->period.period.tv_usec*1000 )   ;
        tsnorm ( &t );
    }

    Logger::info(Logger::Severity::HIGH) << "Cleanly exiting NRT thread: " << ( *th_hook ).name << Logger::endl(); 

    return 0;

}



// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
