#include <XBotCore-interfaces/XBotRT_ipc.h>

#include <sys/mman.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <malloc.h>
#include <fcntl.h>
#include <errno.h>


static void fail ( const char *reason ) {
    perror ( reason );
    exit ( EXIT_FAILURE );
}


int xddp_bind ( const char * label, size_t local_poolsz ) {
    struct rtipc_port_label plabel;
    struct sockaddr_ipc saddr;
    int s;
    struct timeval tv;

    /* Get a datagram socket to bind to the RT endpoint. Each
     * endpoint is represented by a port number within the XDDP
     * protocol namespace. */
    if ( ( s=socket ( AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP ) ) < 0 )
        fail ( "socket" );


    /* Set a port label. This name will be registered when
     * binding, in addition to the port number (if given). */
    strcpy ( plabel.label, label );
    //strcpy(port_label, label);
    if ( setsockopt ( s, SOL_XDDP, XDDP_LABEL, &plabel, sizeof ( plabel ) ) )
        fail ( "setsockopt xddp_label" );
    /*
     * Set a local local_poolsz pool for the RT endpoint. Memory needed to
     * convey datagrams will be pulled from this pool, instead of
     * Xenomai's system pool.
     */
    if ( local_poolsz > 0 ) {
        if ( setsockopt ( s, SOL_XDDP, XDDP_POOLSZ, &local_poolsz, sizeof ( local_poolsz ) ) )
            fail ( "setsockopt xddp_poolsz" );
    }

    /*
     * Bind the socket to the port, to setup a proxy to channel
     * traffic to/from the Linux domain. Assign that port a label,
     * so that peers may use a descriptive information to locate
     * it. For instance, the pseudo-device matching our RT
     * endpoint will appear as
     * /proc/xenomai/registry/rtipc/xddp/<XDDP_PORT_LABEL> in the
     * Linux domain, once the socket is bound.
     *
     * saddr.sipc_port specifies the port number to use. If -1 is
     * passed, the XDDP driver will auto-select an idle port.
     */
    memset ( &saddr, 0, sizeof ( saddr ) );
    saddr.sipc_family = AF_RTIPC;
    saddr.sipc_port = -1;
    if ( bind ( s, ( struct sockaddr * ) &saddr, sizeof ( saddr ) ) ) {
        printf("Error %s %s\n", __FUNCTION__, label);
        fail ( "bind" );
    }

    return s;
}

int xddp_connect ( const char * label ) {
    struct rtipc_port_label plabel;
    //char port_label[XDDP_LABEL_LEN];
    struct sockaddr_ipc saddr;
    struct timeval tv;
    int s;
    socklen_t addrlen;

    if ( ( s=socket ( AF_RTIPC, SOCK_DGRAM, IPCPROTO_XDDP ) ) < 0 )
        fail ( "socket" );

    /*
     * Set the socket timeout; it will apply when attempting to
     * connect to a labeled port, and to recvfrom() calls.  The
     * following setup tells the XDDP driver to wait for at most
     * one second until a socket is bound to a port using the same
     * label, or return with a timeout error.
     */
    tv.tv_sec = 1;
    tv.tv_usec = 0;
    if ( setsockopt ( s, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof ( tv ) ) )
        fail ( "setsockopt" );

    /*
     * Set a port label. This name will be used to find the peer
     * when connecting, instead of the port number.
     */
    strcpy ( plabel.label, label );
    //strcpy(port_label, label);
    if ( setsockopt ( s, SOL_XDDP, XDDP_LABEL, &plabel, sizeof ( plabel ) ) )
        //if (setsockopt(s, SOL_RTIPC, XDDP_SETLABEL, &port_label, sizeof(port_label)))
        fail ( "setsockopt" );

    memset ( &saddr, 0, sizeof ( saddr ) );
    saddr.sipc_family = AF_RTIPC;
    saddr.sipc_port = -1;   /* Tell XDDP to search by label. */
    if ( connect ( s, ( struct sockaddr * ) &saddr, sizeof ( saddr ) ) )
        fail ( "connect" );

    /*
     * We succeeded in making the port our default destination
     * address by using its label, but we don't know its actual
     * port number yet. Use getpeername() to retrieve it.
     */
    addrlen = sizeof ( saddr );
    if ( getpeername ( s, ( struct sockaddr * ) &saddr, &addrlen ) || addrlen != sizeof ( saddr ) )
        fail ( "getpeername" );

    rt_printf ( "%s: NRT peer is reading from /dev/rtp%d\n", __FUNCTION__, saddr.sipc_port );

    return s;
}
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
