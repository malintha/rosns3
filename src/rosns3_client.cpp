// #include "rosns3_client/utils.h"
#include "client.h"
#include "ros/console.h"
#include "iostream"
#include "string.h"

namespace utils = clientutils;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "~");
    ros::NodeHandle n;
    utils::params_t params;

    params.n_backbone = std::atoi(argv[1]);
    int n_ue = std::atoi(argv[2]);
    params.n_robots = params.n_backbone + n_ue;
    params.port = std::atoi(argv[3]);
    params.frequency = std::atof(argv[4]);
    params.hops_k = std::atoi(argv[5]);
    
    ROS_DEBUG_STREAM("server_port: " << params.port << " frequency: "
                                     << params.frequency);

    Client client(params, n);
    client.run();
    
    // publish the routing tables to a service
}