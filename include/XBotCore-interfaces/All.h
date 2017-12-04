/*
 * Copyright (C) 2016 IIT-ADVR
 * Author: Luca Muratore
 * email:  luca.muratore@iit.it
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

#ifndef __XBOTCORE_ALL_H__
#define __XBOTCORE_ALL_H__

#include <XBotCore-interfaces/IXBotModel.h>
#include <XBotCore-interfaces/IXBotJoint.h>
#include <XBotCore-interfaces/IXBotFT.h>
#include <XBotCore-interfaces/IXBotIMU.h>
#include <XBotCore-interfaces/IXBotHand.h>
#include <XBotCore-interfaces/XBotPlugin.h>
#include <XBotCore-interfaces/XBotSharedMemory.h>
#include <XBotCore-interfaces/XBotSharedObject.h>
#include <XBotCore-interfaces/XDomainCommunication.h>
#include <XBotCore-interfaces/XBotPipes.h>
#include <XBotCore-interfaces/XBotESC.h>
#include <XBotCore-interfaces/XBotTransmission.h>

#if defined( __XENO__ ) || defined( __COBALT__ )
#include <XBotCore-interfaces/XBotRT_ipc.h>
#endif

#endif