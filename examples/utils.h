#include <vector>
#include "ns3/rosns3-helper.h"
#include <eigen3/Eigen/Dense>

using namespace ns3;
using namespace Eigen;

namespace utils {
    std::vector<mobile_node_t> sort_ue(std::vector<mobile_node_t> ue);
    std::vector<mobile_node_t> get_ue(Vector2d roi_means, Vector2d roi_vars);

    // sort ue from the distanc to the center of ROI
}