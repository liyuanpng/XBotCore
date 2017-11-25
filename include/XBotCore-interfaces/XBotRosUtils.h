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
#include <queue>
#include <assert.h>

#include <XBotInterface/Thread.h>
#include <XBotInterface/RtLog.hpp>
#include <XBotInterface/Utils.h>

#include <ros/ros.h>
#include <ros/topic_manager.h>
#include <ros/transport_hints.h>




namespace XBot {
    
    namespace RosUtils {
        
        
        class RosHandle;

        
        class PublisherWrapper {
            
        public:
            
            friend class RosHandle;
            
            typedef std::shared_ptr<PublisherWrapper> Ptr;
            
           
            PublisherWrapper():
                QUEUE_SIZE(0),
                _tail(-1)
            {
                _mutex.reset(new Mutex);
            }
            
            template <typename MessageType>
            void pushToQueue(const MessageType& msg)
            {
                if(!_mutex->try_lock()){
                    return;
                }
                
                
                _msg_queue.push_back();
                ros::SerializedMessage& current_ser_msg = _msg_queue.back();
                
                int len = ros::serialization::serializationLength(msg) + 4;
                
                if(len > current_ser_msg.num_bytes){
                    Logger::info("RosUtils::PublisherWrapper resetting buffer: %d > %d\n", len, current_ser_msg.num_bytes);
                    current_ser_msg.buf.reset(new uint8_t[len]);
                    current_ser_msg.num_bytes = len;
                    current_ser_msg.message_start = current_ser_msg.buf.get();
                    
                }
                
                ros::serialization::OStream s(current_ser_msg.buf.get(), (uint32_t)current_ser_msg.num_bytes);
                ros::serialization::serialize(s, (uint32_t)current_ser_msg.num_bytes - 4);
                current_ser_msg.message_start = s.getData();
                ros::serialization::serialize(s, msg);
                
                _mutex->unlock();
                
            }
            
            void popAndPublish()
            {
                std::lock_guard<Mutex> guard(*_mutex);
                
                while(_msg_queue.size() > 0){
                    
                    ros::TopicManager::instance()->publish(_pub.getTopic(), 
                                                       boost::bind(&PublisherWrapper::get_tail, this),
                                                           _msg_queue.back()
                                                       );
                    
                    _msg_queue.pop_back();
                    
                }
                
            }
            
            void clearQueue()
            {
                std::lock_guard<Mutex> guard(*_mutex);
                
                _msg_queue.reset();
                
            }
            
        private:
            
            PublisherWrapper(ros::Publisher pub, int queue_size = 1):
                _pub(pub),
                QUEUE_SIZE(queue_size),
                _tail(-1),
                _msg_queue(queue_size)
            {
                _mutex.reset(new Mutex);
                
            }
            
            ros::SerializedMessage get_tail()
            {
                return _msg_queue.back();
            }
            
            int QUEUE_SIZE;
            
            XBot::Utils::LimitedDeque<ros::SerializedMessage> _msg_queue;
            std::unique_ptr<Mutex> _mutex;
            int _tail;
            ros::Publisher _pub;
            
        };
        
        
        class SubscriberWrapper {
            
        public:
            
            friend class RosHandle;
            
            typedef std::shared_ptr<SubscriberWrapper> Ptr;
            
        private:
            
            SubscriberWrapper(ros::Subscriber subscriber):
                _sub(subscriber)
            {}
            
            ros::Subscriber _sub;
        };
        
        
        class ServiceServerWrapper {
            
        public:
            
            friend class RosHandle;
            
            typedef std::shared_ptr<ServiceServerWrapper> Ptr;
            
        private:
            
            ServiceServerWrapper(ros::ServiceServer service_server):
                _srv(service_server)
            {}
            
            
            ros::ServiceServer _srv;
        };

        class RosHandle {
        
        public:
            
            typedef std::shared_ptr<RosHandle> Ptr;
            
            
            template <typename MessageType>
            PublisherWrapper::Ptr advertise(std::string topic_name, int queue_size = 1)
            {
                std::lock_guard<Mutex> guard(_mtx);
                
                ros::Publisher pub = _nh.advertise<MessageType>(topic_name, queue_size);
                auto pub_wrapper =  std::shared_ptr<PublisherWrapper>( new PublisherWrapper(pub, queue_size) );
                _ros_pub.push_back(pub_wrapper);
                return pub_wrapper;
            }
            
            template <typename MessageType, typename ObjectType>
            SubscriberWrapper::Ptr subscribe(const std::string& topic_name, 
                                             int queue_size, 
                                                    void(ObjectType::*fp)(const boost::shared_ptr<MessageType const>&), 
                                             ObjectType* obj, 
                                             const ros::TransportHints& transport_hints = ros::TransportHints()
                                             )
            {
                ros::Subscriber sub = _nh.subscribe<MessageType, ObjectType>(topic_name, 
                                                                             queue_size, 
                                                                             fp, 
                                                                             obj, 
                                                                             transport_hints);
                
                return  std::shared_ptr<SubscriberWrapper>( new SubscriberWrapper(sub) );
            }
            
            
            template <typename MessageType>
            SubscriberWrapper::Ptr subscribe(const std::string& topic_name, 
                                             int queue_size, 
                                             const boost::function<void (const boost::shared_ptr<MessageType const>&)>& callback,
                                             const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr(), 
                                             const ros::TransportHints& transport_hints = ros::TransportHints()
                                             )
            {
                ros::Subscriber sub = _nh.subscribe<MessageType>(topic_name, 
                                                                 queue_size, 
                                                                 callback, 
                                                                 tracked_object, 
                                                                 transport_hints);
                
                return  std::shared_ptr<SubscriberWrapper>( new SubscriberWrapper(sub) );
            }
            
            template<class T, class MReq, class MRes>
            ServiceServerWrapper::Ptr advertiseService(const std::string& service, 
                                                       bool(T::*srv_func)(MReq &, MRes &), 
                                                       T *obj)
            {
                auto srv = _nh.advertiseService<T, MReq, MRes>(service, srv_func, obj);
                
                return std::shared_ptr<ServiceServerWrapper>(new ServiceServerWrapper(srv));
            }
            
            
            template<class MReq, class MRes>
            ServiceServerWrapper::Ptr advertiseService(const std::string& service, 
                                                       const boost::function<bool(MReq&, MRes&)>& callback, 
                                                       const ros::VoidConstPtr& tracked_object = ros::VoidConstPtr())
            {
                auto srv = _nh.advertiseService<MReq, MRes>(service, callback, tracked_object);
                
                return std::shared_ptr<ServiceServerWrapper>(new ServiceServerWrapper(srv));
            }
            
            
            void publishAll()
            {
                std::lock_guard<Mutex> guard(_mtx);
                
                for(PublisherWrapper::Ptr p : _ros_pub){
                    p->popAndPublish();
                }
            }
            
            
        private:
            
            ros::NodeHandle _nh;
            
            std::vector<PublisherWrapper::Ptr> _ros_pub;
            
            Mutex _mtx;
            
        };



    }
    
}

#endif //__XBOT_HANDLE_H__

