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
    utils::params_t params = utils::load_params(n);

    ROS_DEBUG_STREAM("server_port: " << params.port << " frequency: "
                                     << params.frequency);

    Client client(params, n);
    client.run();
    
    // publish the routing tables to a service
}