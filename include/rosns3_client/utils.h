#include "iostream"
#include "ros/ros.h"
#include "simulator_utils/Drone.h"

namespace clientutils
{    
    class Node : public Drone
    {
        public:
        Node(int id, ros::NodeHandle n) : Drone(id, n) {};
    };

    typedef struct params
    {
        int port;
        double frequency;
        int n_robots;
        int n_backbone;
        std::string topic_prefix;
    } params_t;

    typedef std::vector<Node*> nodes_t;

    params_t load_params(ros::NodeHandle);


    typedef struct neighborhood_t {
        int id;
        std::vector<int> neighbors;
        // neighborhood_t(int id, std::vector<int> neighbors);
    } neighborhood_t;

};
