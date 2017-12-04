#include <stdio.h>
#include <stdlib.h>

#include <XCM/XBotThread.h>
#include <XBotInterface/RtLog.hpp>

using XBot::Logger;

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void * XBot::rt_periodic_thread ( Thread_hook_Ptr th_hook ) {
    int             ret = 0;
    struct timespec starttp, periodtp;
#ifdef __COBALT__
    struct itimerspec   period_timer_conf;
#endif
    unsigned long   overruns;

    // thread specific initialization
    th_hook->th_init ( 0 );

    Logger::info() << "THREAD INIT: name = " << th_hook->name << ", period " << th_hook->period.period.tv_usec << " us" << Logger::endl();

#ifdef __COBALT__
    ret = pthread_setname_np ( pthread_self(), th_hook->name );
#else
    ret = pthread_set_name_np ( pthread_self(), th_hook->name );
#endif
    
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
#ifdef __COBALT__
    ret = pthread_setmode_np ( 0, PTHREAD_WARNSW, 0);
#else
    ret = pthread_set_mode_np ( 0, PTHREAD_WARNSW);
#endif
    
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    periodtp.tv_sec = 0ULL;
    periodtp.tv_nsec = th_hook->period.period.tv_usec * 1000ULL;
    clock_gettime ( CLOCK_REALTIME, &starttp );
    starttp.tv_nsec += periodtp.tv_nsec;
    tsnorm ( &starttp );

#ifdef __XENO__
    ret = pthread_make_periodic_np ( pthread_self(), &starttp, &periodtp );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_make_periodic_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }
#endif

#ifdef __COBALT__

    th_hook->fd_timer = timerfd_create(CLOCK_MONOTONIC, 0);
    if ( th_hook->fd_timer == -1 ) {
        DPRINTF ( "%s : timerfd_create() return code %d\n",
                  th_hook->name, errno );
        exit ( 1 );        
    }

    clock_gettime ( CLOCK_MONOTONIC, &starttp );
    starttp.tv_sec  += th_hook->period.period.tv_sec;
    starttp.tv_nsec += th_hook->period.period.tv_usec * 1000ULL;
    tsnorm ( &starttp );

    period_timer_conf.it_value = starttp;
    period_timer_conf.it_interval.tv_sec =  th_hook->period.period.tv_sec;
    period_timer_conf.it_interval.tv_nsec = th_hook->period.period.tv_usec * 1000ULL;
    if ( timerfd_settime( th_hook->fd_timer, TFD_TIMER_ABSTIME, &period_timer_conf, NULL) == -1 )
    {
        DPRINTF ( "%s : timerfd_settime() return code %d\n",    
                  th_hook->name, errno );
        exit ( 1 );        
    }
    
    
#endif

    Logger::success(Logger::Severity::HIGH) << "Thread " << th_hook->name << ": start looping" << Logger::endl();

#if defined(__XENO__)
    
    while ( th_hook->_run_loop ) {

        // return 0 if the period expires as expected
        /* - EPERM, the calling context is invalid;
         * - EWOULDBLOCK, the calling thread is not periodic;
         * - EINTR, this service was interrupted by a signal;
         * - ETIMEDOUT, at least one overrun occurred.
         */
        ret = pthread_wait_np ( &overruns );

        switch ( ret ) {
        case ETIMEDOUT :
            DPRINTF ( "%s : pthread_wait_np() ETIMEDOUT %lu\n", th_hook->name, overruns );
            break;
        case EPERM :
        case EWOULDBLOCK :
        case EINTR :
            DPRINTF ( "%s : pthread_wait_np() return code %d\n", th_hook->name, ret );
            exit ( 1 );
        }

        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while
    
#elif defined(__COBALT__)

    while ( th_hook->_run_loop ) {

        uint64_t ticks;
        ret = read( th_hook->fd_timer, &ticks, sizeof(ticks));
        if ( ret < 0 ) {
            printf( "fd_timer wait period failed for thread: err %d\n", ret );
        }
        if ( ticks > 1 ) {
            printf( "fd_timer wait period missed for thread: overruns: %lu\n", (long unsigned int)ticks );
        }
        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while


#endif


    return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void * XBot::rt_non_periodic_thread ( Thread_hook_Ptr th_hook ) {
    int ret = 0;

    // thread specific initialization
    th_hook->th_init ( 0 );

    Logger::info() << "THREAD INIT: name = " << th_hook->name << ", period " << th_hook->period.period.tv_usec << " us" << Logger::endl();

#ifdef __COBALT__
    ret = pthread_setname_np ( pthread_self(), th_hook->name );
#else
    ret = pthread_set_name_np ( pthread_self(), th_hook->name );
#endif
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
#ifdef __COBALT__
    ret = pthread_setmode_np ( 0, PTHREAD_WARNSW, 0);
#else 
    ret = pthread_set_mode_np ( 0, PTHREAD_WARNSW);
#endif
    
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    Logger::success(Logger::Severity::HIGH) << "Thread " << th_hook->name << ": start looping" << Logger::endl();

    while ( th_hook->_run_loop ) {

        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while
    
    Logger::info(Logger::Severity::HIGH) << "Cleanly exiting RT thread: " << ( *th_hook ).name << Logger::endl(); 

    return 0;
}



// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
