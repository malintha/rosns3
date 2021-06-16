#include <vector>
#include "ns3/rosns3-helper.h"
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/v4ping-helper.h"

using namespace ns3;

class CoModel {
    public:
        CoModel(std::vector<mobile_node_t> mobile_nodes, int sim_time, bool use_real_time);
        void run();
        std::vector<mobile_node_t> mobile_nodes;
        void report(std::ostream &);
        void update_mobility_model(std::vector<mobile_node_t> mobile_nodes);

    private:
        uint32_t n_nodes;
        NodeContainer node_container;
        NetDeviceContainer devices;
        Ipv4InterfaceContainer interfaces;
        MobilityHelper mobility;
        bool pcap, print_routes;
        int sim_time;
        bool use_real_time;
        void create_nodes();
        void create_devices();
        void install_applications();
        void install_inet_stack();
        void create_mobility_model();
        std::thread* simulator;
};
