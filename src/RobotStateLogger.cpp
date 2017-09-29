#include <ros/ros.h>
#include <XBotInterface/RobotInterface.h>
#include <XCM/JointStateAdvr.h>
#include <functional>

void callback(XCM::JointStateAdvr::ConstPtr rec_msg, XCM::JointStateAdvr& msg){
 
    msg = *rec_msg;
    
}

int main(int argc, char** argv){
    
    if(argc < 2){
        std::cout << "Usage: " << argv[0] << " path_to_cfg" << std::endl;
        return -1;
    }
    
    ros::init(argc, argv, "robot_state_logger");
    ros::NodeHandle nh;
    
    auto model = XBot::ModelInterface::getModel(argv[1]);
    
    XCM::JointStateAdvr js_msg;
    
    auto js_callback = std::bind(callback, std::placeholders::_1, std::ref(js_msg));
    
    ros::Subscriber js_sub = nh.subscribe<XCM::JointStateAdvr>("/xbotcore/"+model->getUrdf().getName()+"/joint_state", 
                                                               1, 
                                                               js_callback);
    
    while(js_msg.name.size() == 0){
        std::cout << "Waiting for CommunicationHandler to start..." << std::endl;
        ros::Duration(1).sleep();
    }
    
    
    auto robot = XBot::RobotInterface::getRobot(argv[1]);
    
    auto logger = XBot::MatLogger::getLogger("/tmp/RobotStateLog");
    
    int log_capacity = 1e5;
    
    robot->initLog(logger, log_capacity);
    
    ros::Rate loop_rate(200);
    
    std::vector<double> fault(robot->getJointNum(), 0);
    
    std::cout << "Started logging..." << std::endl;
    
    while(ros::ok()){
        
        robot->sense(false);
        
        robot->log(logger, ros::Time::now().toSec());
        
        for(int i = 0; i < js_msg.name.size(); i++){
            auto jname = js_msg.name[i];
            fault[robot->getDofIndex(jname)] = js_msg.fault[i];  
        }
        
        logger->add("fault", fault, 1, log_capacity);
        
    }
    
    
    return 0;   
    
}