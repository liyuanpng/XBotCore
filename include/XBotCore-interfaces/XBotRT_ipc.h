#ifndef __RT_IPC_H__
#define __RT_IPC_H__

#include <stddef.h>

#include <rtdk.h>
#include <rtdm/rtipc.h>

/*
XDDP stands for "cross-domain datagram protocol", i.e. to exchange
datagrams between the Xenomai (primary) real-time domain, and the Linux
realm. This is what the message pipe fans may want to have a look at.
Basically, it connects a real-time RTDM socket to one of the /dev/rtp*
pseudo-devices. The network port used on the socket side matches the
minor device number used on the non RT side. The added bonus of XDDP is
that people relying on the POSIX skin may now have access to the message
pipe feature, without dragging in bits of the native skin API for that
purpose.
*/

#ifdef __cplusplus
extern "C" {
#endif


int xddp_bind ( const char * label, size_t poolsz );

int xddp_connect ( const char * label );

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
// kate: indent-mode cstyle; indent-width 4; replace-tabs on; 
