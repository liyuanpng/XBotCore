#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <vector>
#include <string>

void fs(const std::string& s);

void fc(const XBot::Command& c);

int main(int argc, char **argv){


    XBot::PublisherNRT<Eigen::Vector3d> pub_vec("nrt_to_rt_vec3d");
    XBot::SubscriberNRT<Eigen::Vector3d> sub_vec("rt_to_nrt_vec3d");

    XBot::PublisherNRT<XBot::Command> pub_str("nrt_to_rt_string");
    XBot::SubscriberNRT<XBot::Command> sub_str("rt_to_nrt_string");

    Eigen::Vector3d vec1, vec2;
    std::vector<std::string> strings_to_send = {"HELLO", "I AM", "A NON-REALTIME", "PROCESS", "HOW", "ARE YOU?"};


    for( int it = 0; it < 1e10; it++ ){

        std::cout << "NRT looping..." << std::endl;

        vec1.setConstant(it);
        pub_vec.write(vec1);

        pub_str.write(strings_to_send[it%strings_to_send.size()]);

        if(sub_vec.read(vec2)){
            std::cout << "NRT process: received vector " << vec2.transpose() << std::endl;
        }


        XBot::Command str;
        if(sub_str.read(str)){
            std::cout << "NRT process: received string " << str.str() << std::endl;
        }

        sleep(1);


    }



}