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

#ifndef __XBOT_ROS_UTILS_H__
#define __XBOT_ROS_UTILS_H__

#include <vector>
#include <memory>

#include <XBotInterface/Thread.h>
#include <XBotInterface/RtLog.hpp>

#include <ros/ros.h>
#include <ros/topic_manager.h>


namespace XBot {
    
    namespace RosUtils {

        
        class PublisherWrapper {
            
        public:
            
            typedef std::shared_ptr<PublisherWrapper> Ptr;
            
            PublisherWrapper(ros::Publisher pub, int queue_size = 1):
                _pub(pub),
                QUEUE_SIZE(queue_size),
                _tail(-1)
            {
                _msg_queue.reserve(queue_size);
                _mutex.reset(new Mutex);
                
            }
            
           
            PublisherWrapper():
                QUEUE_SIZE(0),
                _tail(-1)
            {
                _mutex.reset(new Mutex);
            }
            
            template <typename MessageType>
            void pushToQueue(const MessageType& msg)
            {
                std::lock_guard<Mutex> guard(*_mutex);
                
                _tail++;
                
                if(_msg_queue.size() <= _tail){
                    Logger::info(Logger::Severity::HIGH, "Expanding queue to %d", _msg_queue.size()+1);
                    _msg_queue.emplace_back();
                }
                
                ros::SerializedMessage& current_ser_msg = _msg_queue.at(_tail);
                
                int len = ros::serialization::serializationLength(msg) + 4;
                
                if(len > current_ser_msg.num_bytes){
                    Logger::info(Logger::Severity::HIGH, "Malloc because %d > %d", len, current_ser_msg.num_bytes);
                    current_ser_msg.buf.reset(new uint8_t[len]);
                    current_ser_msg.num_bytes = len;
                    current_ser_msg.message_start = current_ser_msg.buf.get();
                    
                }
                
                ros::serialization::OStream s(current_ser_msg.buf.get(), (uint32_t)current_ser_msg.num_bytes);
                ros::serialization::serialize(s, (uint32_t)current_ser_msg.num_bytes - 4);
                current_ser_msg.message_start = s.getData();
                ros::serialization::serialize(s, msg);
                
            }
            
            void popAndPublish()
            {
                std::lock_guard<Mutex> guard(*_mutex);
                
                while(_tail > -1){
                    
                    ros::TopicManager::instance()->publish(_pub.getTopic(), 
                                                       boost::bind(&PublisherWrapper::get_tail, this),
                                                           _msg_queue.back()
                                                       );
                    
                    _tail--;
                    
                }
                
            }
            
            void clearQueue()
            {
                std::lock_guard<Mutex> guard(*_mutex);
                
                _tail = -1;
                
            }
            
        private:
            
            ros::SerializedMessage get_tail()
            {
                return _msg_queue.at(_tail);
            }
            
            int QUEUE_SIZE;
            
            std::vector< ros::SerializedMessage > _msg_queue;
            std::unique_ptr<Mutex> _mutex;
            int _tail;
            ros::Publisher _pub;
            
        };
        
        
        class Subscriber {
            
        public:
            
            typedef std::shared_ptr<Subscriber> Ptr;
        };

        class RosUtils {
        
        public:
            
            
            
        private:
            
        };



    }
    
}

#endif //__XBOT_HANDLE_H__

