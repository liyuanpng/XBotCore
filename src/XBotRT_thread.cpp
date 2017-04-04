#include <stdio.h>
#include <stdlib.h>

#include <XCM/XBotThread.h>

///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void * rt_periodic_thread ( Thread_hook_Ptr th_hook ) {
    int             ret = 0;
    struct timespec starttp, periodtp;
    unsigned long   overruns;

    // thread specific initialization
    th_hook->th_init ( 0 );

    DPRINTF ( "THREAD INIT: name = %s, period %ld us\n",
              th_hook->name, th_hook->period.period.tv_usec );

    ret = pthread_set_name_np ( pthread_self(), th_hook->name );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
    ret = pthread_set_mode_np ( 0, PTHREAD_WARNSW );
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

    ret = pthread_make_periodic_np ( pthread_self(), &starttp, &periodtp );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_make_periodic_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    DPRINTF ( "THREAD INIT: start looping ...\n" );

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


    return 0;
}


///////////////////////////////////////////////////////////////////////////////
//
///////////////////////////////////////////////////////////////////////////////

void * rt_non_periodic_thread ( Thread_hook_Ptr th_hook ) {
    int ret = 0;

    // thread specific initialization
    th_hook->th_init ( 0 );

    DPRINTF ( "THREAD INIT: name = %s, period %ld us\n",
              th_hook->name, th_hook->period.period.tv_usec );

    ret = pthread_set_name_np ( pthread_self(), th_hook->name );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_name_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    // PTHREAD_WARNSW, when set, cause the signal SIGXCPU to be sent to the
    // current thread, whenever it involontary switches to secondary mode;
    ret = pthread_set_mode_np ( 0, PTHREAD_WARNSW );
    if ( ret != 0 ) {
        DPRINTF ( "%s : pthread_set_mode_np() return code %d\n",
                  th_hook->name, ret );
        exit ( 1 );
    }

    DPRINTF ( "THREAD INIT: start looping ...\n" );

    while ( th_hook->_run_loop ) {

        // thread specific loop
        th_hook->th_loop ( 0 );

    } // end while

    return 0;
}



// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
