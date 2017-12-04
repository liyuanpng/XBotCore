/*
 * Copyright (C) 2017 IIT-ADVR
 * Author: Arturo Laurenzi, Luca Muratore
 * email:  arturo.laurenzi@iit.it, luca.muratore@iit.it
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>
*/


#ifndef __I_X_BOT_IMU_H__
#define __I_X_BOT_IMU_H__

#include <vector>
#include <XBotInterface/RtLog.hpp>

namespace XBot
{
    class IXBotIMU;
}

/**
 * @brief XBotCore IMU Interface
 *
 */
class XBot::IXBotIMU
{

public:

    virtual bool get_imu(int imu_id, std::vector<double>& lin_acc,
                         std::vector<double>& ang_vel,
                         std::vector<double>& quaternion) = 0;

    virtual bool get_imu_fault(int imu_id, double& fault) = 0;

    virtual bool get_imu_rtt(int imu_id, double& rtt) = 0;

    virtual ~IXBotIMU() {
//         if(Logger::GetVerbosityLevel() == Logger::Severity::LOW)
//             std::cout << __func__ << std::endl;
    };
};

#endif //__I_X_BOT_IMU_H__
