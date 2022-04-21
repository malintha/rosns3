#include "utils.h"
#include <random>
#include <eigen3/Eigen/Dense>
#include <algorithm>

std::vector<mobile_node_t> utils::get_ue(Eigen::Vector2d roi_means, Eigen::Vector2d roi_vars)
{
    int n_ues = 4;
    std::random_device rd{};
    std::mt19937 gen{};
    std::normal_distribution<> d1{roi_means(0), roi_vars(0)};
    std::normal_distribution<> d2{roi_means(1), roi_vars(1)};

    std::vector<mobile_node_t> ue_nodes;

    for (int i = 0; i < n_ues; i++)
    {
        ns3::Vector pose(d1(gen), d2(gen), 0);
        mobile_node_t ue_node = {.position = pose, .id = i};
        ue_nodes.push_back(ue_node);
    }

    typedef std::function<bool(mobile_node_t, mobile_node_t)> Comparator;
    Comparator compFunctor =
        [](mobile_node_t elem1, mobile_node_t elem2)
    {
        return elem1.position.GetLength() < elem2.position.GetLength();
    };
    std::sort(ue_nodes.begin(), ue_nodes.end(), compFunctor);
    return ue_nodes;
}
std::vector<mobile_node_t> utils::get_ue(std::vector<mobile_node_t> mobile_nodes, int backbone_nodes)
{
    std::vector<mobile_node_t> ue_nodes;
    for (int i = backbone_nodes; i < mobile_nodes.size(); i++)
    {
        ue_nodes.push_back(mobile_nodes[i]);
        // std::cout << "UE Locations: "
        //           << "id: " << i << " pose: " << mobile_nodes[i].position<<std::endl;
    }
    return ue_nodes;
}

std::vector<std::pair<mobile_node_t, mobile_node_t>> utils::get_farthest_pairs(std::vector<mobile_node_t> ue)
{
    std::vector<std::pair<mobile_node_t, mobile_node_t>> ue_pairs;
    // std::vector<mobile_node_t> ue_pair(2);

    // std::vector<int>::iterator first = myints.begin(), last = myints.end();

    //  while((*first) != n-r+1){
    //     std::vector<int>::iterator mt = last;

    //     while (*(--mt) == n-(last-mt)+1);
    //     (*mt)++;
    //     while (++mt != last) *mt = *(mt-1)+1;

    //     std::for_each(first, last, myfunction);
    //     std::cout << std::endl;
    // }
    return ue_pairs;
}

void utils::get_hop_stats(std::vector<int> hops) {
    
}

typedef std::function<bool(std::pair<double, uint16_t>, std::pair<double, uint16_t>)> double_pair_comparator;
double_pair_comparator double_pair_comp_functor =
    [](std::pair<double, uint16_t> disA, std::pair<double, uint16_t> disB)
    {
        return disA.first < disB.first;
    };
typedef std::function<bool(double, double)> double_comparator;

double_comparator double_comp_functor =
    [](double disA, double disB)
    {
        return disA > disB;
    };
int utils::get_closest_uav(ns3::Vector ue_pos, NodeContainer backbone) {
    std::vector<std::pair<double, uint16_t> > distances;
    for (uint16_t i = 0; i < backbone.GetN(); i++)
    {
        Ptr<Node> uav_node = backbone.Get(i);
        Ptr<MobilityModel> mob = uav_node->GetObject<MobilityModel>();
        ns3::Vector uav_pos = mob->GetPosition();
        double dis = CalculateDistance(uav_pos, ue_pos);
        distances.push_back(std::pair<double, uint16_t>(dis, i));
    }
    std::sort(distances.begin(), distances.end(), double_pair_comp_functor);
    int idx_closest = distances[0].second;
    distances.clear();
    return idx_closest;
}

double utils::get_rss(ns3::Vector &ue_pos, NodeContainer &backbone, Ptr<PropagationLossModel> &model) {
    std::vector<double> rss_vals;
    // double sum = 0;
    Ptr<ConstantPositionMobilityModel> ue_pos_mob = CreateObject<ConstantPositionMobilityModel> ();
    ue_pos_mob->SetPosition(ue_pos);
    for (uint16_t i = 0; i < backbone.GetN(); i++)
    {
        Ptr<Node> uav_node = backbone.Get(i);
        Ptr<MobilityModel> uav_mob = uav_node->GetObject<MobilityModel>();
        // Vector uav_pos = mob->GetPosition();
        double rss = model->CalcRxPower(16.02, ue_pos_mob, uav_mob);
        rss_vals.push_back(rss);
        // sum+=rss;
    }
    std::sort(rss_vals.begin(), rss_vals.end(), double_comp_functor);
    // double rss_sel = sum/backbone.GetN();
    double rss_sel = rss_vals[0];
    return rss_sel;
}

