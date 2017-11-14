#include <XBotCore-interfaces/XBotRosUtils.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/JointState.h>

int main(int argc, char** argv){
    
    ros::init(argc, argv, "example_rosutils");
    
    ros::NodeHandle nh;
    
    XBot::RosUtils::PublisherWrapper pub_rt, pub_rt_1;
    
    pub_rt = XBot::RosUtils::PublisherWrapper(nh.advertise<geometry_msgs::Point>("test_topic", 10), 10);
    pub_rt_1 = XBot::RosUtils::PublisherWrapper(nh.advertise<sensor_msgs::JointState>("test_topic_1", 10), 10);
    
    while(ros::ok()){
        
        for(int i : {1, 2, 3}){
            geometry_msgs::Point msg;
            msg.x = ros::Time::now().toSec();
            msg.y = 2*i;
            msg.z = 3.1415;
            pub_rt.pushToQueue(msg);
            
            sensor_msgs::JointState msg1;
            msg1.name.push_back("Io");
            msg1.position.push_back(0);

            

            msg1.name.push_back("Sono");
            msg1.position.push_back(2.73*i);
            

            msg1.name.push_back("Leggenda");
            msg1.position.push_back(3.14);
            
            pub_rt_1.pushToQueue(msg1);
                        
        }
        
        
        
        pub_rt.popAndPublish();
        pub_rt_1.popAndPublish();
        
        ros::Duration(3.0).sleep();
    }
    
    
    return 0;
    
}