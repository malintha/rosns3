#include <vector>
#include "ns3/rosns3-helper.h"
#include <eigen3/Eigen/Dense>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/constant-position-mobility-model.h"

using namespace ns3;
using namespace Eigen;

namespace utils {
    std::vector<mobile_node_t> sort_ue(std::vector<mobile_node_t> ue);
    std::vector<mobile_node_t> get_ue(Eigen::Vector2d roi_means, Eigen::Vector2d roi_vars);
    std::vector<mobile_node_t> get_ue(std::vector<mobile_node_t> mobile_nodes, int backbone_nodes);

    std::vector<std::pair<mobile_node_t,mobile_node_t> > get_farthest_pairs(std::vector<mobile_node_t> ue);
    std::vector<mobile_node_t> get_ue(std::vector<mobile_node_t> mobile_nodes, int backbone_nodes);
    void get_hop_stats(std::vector<int> hops);
    int get_closest_uav(ns3::Vector ue_pos, NodeContainer backbone);
    double get_rss(ns3::Vector &ue_pos, NodeContainer &backbone, Ptr<PropagationLossModel> &model);

}