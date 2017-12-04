#ifndef __SONO_LA_GUARDIA_DI_ROS_SERIALIZATION_H__
#define __SONO_LA_GUARDIA_DI_ROS_SERIALIZATION_H__


#include <ros/ros.h>

namespace XBot {

    template <typename RosMessageType, int MaxSizeBytes = 4096>
    class RosMessageWrapper {

    public:

        RosMessageWrapper():_is_valid(false), _serialization_length(0) {}

        bool wrap(const RosMessageType& ros_msg)
        {
            /* Get message length in bytes */
            _serialization_length = ros::serialization::serializationLength(ros_msg);

            /* If message exceeds buffer size, return */
            if( _serialization_length > MaxSizeBytes ){

                _serialization_length = 0;
                _is_valid = false;

                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << "! Message length exceeds RosMessageWrapper max size \
                (" << _serialization_length << " > " << MaxSizeBytes << ")! Set the second template parameter to a value \
                greater than " << _serialization_length << ", and take care that on the other side of the pipe the same \
                value is used!" << std::endl;
                return false;
            }

            ros::serialization::OStream stream(_bytes, _serialization_length);
            ros::serialization::serialize(stream, ros_msg);
            _is_valid = true;

            return true;
        }

        bool getMessage(RosMessageType& msg)
        {
            if( !_is_valid ){
                std::cerr << "ERROR in " << __PRETTY_FUNCTION__ << " Invalid message :(" << std::endl;
                return false;
            }

            ros::serialization::IStream stream(_bytes, _serialization_length);
            ros::serialization::Serializer<RosMessageType>::read(stream, msg);

            return true;

        }

        int getMaxSize() { return MaxSizeBytes; }



    private:

        bool _is_valid;
        uint8_t _bytes[MaxSizeBytes];
        uint32_t _serialization_length;

    };

}

#endif
