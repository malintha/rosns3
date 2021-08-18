#include "utils.h"
#include <random>
#include <eigen3/Eigen/Dense>
#include <algorithm>

std::vector<mobile_node_t> utils::get_ue(Vector2d roi_means, Vector2d roi_vars) {
    int n_ues = 10;
    std::random_device rd{};
    std::mt19937 gen{};
    std::normal_distribution<> d1{roi_means(0),roi_vars(0)};
    std::normal_distribution<> d2{roi_means(1),roi_vars(1)};

    std::vector<mobile_node_t> ue_nodes;

    for(int i=0; i<n_ues; i++) {
        Vector pose(d1(gen), d2(gen), 0); 
        mobile_node_t ue_node = {.position=pose, .id=i};
        // NS_LOG_INFO("STA Locations: "<< "id: "<<i << " pose: "<< pose );
        ue_nodes.push_back(ue_node);   
    }

    typedef std::function<bool(mobile_node_t, mobile_node_t)> Comparator;
    Comparator compFunctor =
            [](mobile_node_t elem1, mobile_node_t elem2) {
                return elem1.position.GetLength() < elem2.position.GetLength();
            };
    std::sort(ue_nodes.begin(), ue_nodes.end(), compFunctor);
    return ue_nodes;

}

std::vector<std::pair<mobile_node_t,mobile_node_t> > utils::get_farthest_pairs(std::vector<mobile_node_t> ue){
    std::vector<std::pair<mobile_node_t,mobile_node_t> > ue_pairs;
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
