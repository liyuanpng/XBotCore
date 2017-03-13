#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>

int main(int argc, char **argv){


    XBot::PublisherRT<Eigen::Vector3d> pub_vec("rt_to_nrt_vec3d");
    XBot::SubscriberRT<Eigen::Vector3d> sub_vec("nrt_to_rt_vec3d");

    XBot::PublisherRT<XBot::Command> pub_str("rt_to_nrt_string");
    XBot::SubscriberRT<XBot::Command> sub_str("nrt_to_rt_string");

    Eigen::Vector3d vec1, vec2;
    std::vector<std::string> strings_to_send = {"HELLO THERE!", "I AM", "A HARD-REALTIME", "PROCESS", "I AM", "FINE"};


    for( int it = 0; it < 1e10; it++ ){



        vec1.setConstant(it*2);
        pub_vec.write(vec1);

        std::cout << "RT looping...publishing "<< vec1.transpose() << std::endl;

        pub_str.write(strings_to_send[it%strings_to_send.size()]);

        if(sub_vec.read(vec2)){
            std::cout << "RT process: received vector " << vec2.transpose() << std::endl;
        }


        XBot::Command str;
        if(sub_str.read(str)){
            std::cout << "RT process: received string " << str.str() << std::endl;
        }

        usleep(100000);


    }



}