// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Copyright (C) 2013  iCub Facility, Istituto Italiano di Tecnologia
 * Author: Alberto Cardellino
 * email: alberto.cardellino@iit.it
 * Permission is granted to copy, distribute, and/or modify this program
 * under the terms of the GNU General Public License, version 2 or any
 * later version published by the Free Software Foundation.
 *
 * A copy of the license can be found at
 * http://www.robotcub.org/icub/license/gpl.txt
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General
 * Public License for more details
 */

//#ifndef __YARP_VN100__
//#define __YARP_VN100__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/GenericSensorInterfaces.h>
#include <yarp/os/Stamp.h>
#include <yarp/os/Time.h>
#include <yarp/os/Semaphore.h>
#include <yarp/dev/PreciselyTimed.h>
#include <yarp/os/RateThread.h>
#include <string.h>
#include <termios.h> // terminal io (serial port) interface

#include "vn100.h"

namespace yarp{
    namespace dev{
        class VN100;
    }
}


/**
 *
 * @ingroup dev_impl
 *
 * Driver for VN-100 imu from VectorNav (http://www.vectornav.com/products/vn100-rugged)
 * @author Alberto Cardellino
 */
class yarp::dev::VN100 :       public yarp::dev::IGenericSensor,
                                public yarp::dev::IPreciselyTimed,
                                public yarp::dev::DeviceDriver
{
private:
    // communication data
    int             fd_serial_port;
    std::string     comPortName;
    int             com_baudrate;

    int             counter;
    int             nchannels;

    // data specific for this imu
    Vn100 vn100;
    VN_ERROR_CODE   errorCode;
    VnDeviceCompositeData  imu_data;
    
    // Data structure specific for each command
    yarp::os::Semaphore data_mutex;
    yarp::os::Semaphore sync_mutex;

    yarp::os::Stamp     lastStamp;

public:
    VN100();
    ~VN100();

    // Device Driver interface
    virtual bool open(yarp::os::Searchable &config);
    virtual bool close();

    // IGenericSensor interface.
    /**
     * @brief read data from IMU
     * 
     * @param out 
     * 0  1  2  = Euler orientation data (Kalman filter processed)
     * 3  4  5  = Calibrated 3-axis (X, Y, Z) acceleration data
     * 6  7  8  = Calibrated 3-axis (X, Y, Z) gyroscope data
     * 9 10  11 = Calibrated 3-axis (X, Y, Z) magnetometer data
     * @return true on success
     */
    virtual bool read(yarp::sig::Vector &out);
    virtual bool getChannels(int *nc);
    virtual bool calibrate(int ch, double v);

    virtual yarp::os::Stamp getLastInputStamp();
};


//#endif // __YARP_VN100__
