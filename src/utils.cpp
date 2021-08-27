#include "utils.h"

// namespace utils = clientutils;
// using namespace utils;

clientutils::params_t clientutils::load_params(ros::NodeHandle nh) {
    clientutils::params_t params;
    nh.getParam("rosns3_server_port", params.port);
    nh.getParam("rosns3_server_frequency", params.frequency);
    nh.getParam("rosns3_server_n_robots", params.n_robots);
    nh.getParam("rosns3_server_n_backbone", params.n_backbone);
    nh.getParam("rosns3_server_topic_prefix", params.topic_prefix);

    return params;
};

// clientutils::Node::Node(int id, ros::NodeHandle n):Drone(id, n) {
// };