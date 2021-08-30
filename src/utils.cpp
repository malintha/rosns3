#include "utils.h"
#include <eigen3/Eigen/Dense>
#include <ros/console.h>
#include <strings.h>
#include <fstream>

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
double clientutils::get_avg_dist(std::vector<clientutils::Node*> nodes) {
    std::vector<double> dist;
    for (int i = 0; i < nodes.size(); i++)
    {
        simulator_utils::Waypoint state_i = nodes[i]->get_state();
        Eigen::Vector3d pos_i(state_i.position.x, state_i.position.y, state_i.position.z);
        for (int j = 0; j < nodes.size(); j++)
        {
            if (j == i)
                continue;
            simulator_utils::Waypoint state_j = nodes[j]->get_state();
            Eigen::Vector3d pos_j(state_j.position.x, state_j.position.y, state_j.position.z);
            ;
            dist.push_back((pos_i - pos_j).norm());
        }
    }
    double tot_dis;
    for (double d : dist)
    {
        tot_dis += d;
    }
    // ROS_INFO_STREAM(tot_dis / dist.size() << " " << tot_nbrs / params.n_backbone << " " << min_nbrs << " " << max_nbrs);
    return (tot_dis / dist.size());
}

void clientutils::write_to_file(std::vector<int> hops, double avg_dis) {
    std::ofstream outss;
    std::stringstream ss;
    ss << "/home/malintha/ns3/ns-allinone-3.30/ns-3.30/src/rosns3/scripts/hops.txt";

    outss.open(ss.str(), std::ios_base::app);
    outss << std::endl << avg_dis << " ";
    for (int i = 0; i < hops.size(); i++) {
        outss << " " << hops[i];
    }

    outss.close();
}
