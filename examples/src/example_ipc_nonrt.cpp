#include <XBotCore-interfaces/XDomainCommunication.h>
#include <eigen3/Eigen/Dense>

int main(int argc, char **argv){


    XBot::PublisherNRT<Eigen::Vector3d> pub("rt_to_nrt");
    XBot::SubscriberNRT<Eigen::Vector3d> sub("nrt_to_rt");

    Eigen::Vector3d vec1, vec2;


    for( int it = 0; it < 100; it++ )

        vec1.setConstant(it);
        pub.write(vec1);

        if(sub.read(vec2)){
            std::cout << "NRT process: received vector " << vec2.transpose() << std::endl;
        }

        sleep(1);


    }



}