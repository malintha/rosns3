#include "iostream"
#include "ros/ros.h"
#include "Robot.h"

namespace clientutils
{    
    class Node : public Robot
    {
        public:
            Node(int id, ros::NodeHandle n, bool backbone);
            void set_routing_nodes(std::vector<int> routing_nodes);
            void publish_routing_nodes();
            bool is_backbone();
        private:
            double pub_frequency;
            ros::Publisher routing_pub;
            std::vector<int> routing_nodes;
            bool backbone;
    };

    typedef struct params
    {
        int port;
        double frequency;
        int n_robots;
        int n_backbone;
        std::string topic_prefix;
        int hops_k;
    } params_t;

    typedef std::vector<Node*> nodes_t;

    params_t load_params(ros::NodeHandle);

    double get_avg_dist(std::vector<clientutils::Node*> nodes);

    typedef struct neighborhood_t {
        int id;
        std::vector<int> neighbors;
        // neighborhood_t(int id, std::vector<int> neighbors);
    } neighborhood_t;

    void write_to_file(std::vector<int> hops, double avg_dis);

    int has_value(std::vector<std::pair<int, int>>, int val);
};
