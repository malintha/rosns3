#include "ros/ros.h"
#include "ros/console.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "~");
    double frequency = 1;
    double dt = 1/frequency;
    ros::NodeHandle n;

    int server_port;
    n.getParam("rosns3_server_port", server_port);
    ROS_DEBUG_STREAM("server_port: "<<server_port);
}