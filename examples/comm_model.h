#include <vector>
#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/mobility-module.h"
#include "ns3/v4ping-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/olsr-helper.h"
#include "messages/network_routing_generated.h"

// #include "ns3/udp-echo-helper.h"
#include "utils.h"

using namespace ns3;
using namespace Eigen;

class CoModel {
    public:
        CoModel(std::vector<mobile_node_t> mobile_nodes, int backbone_nodes, int sim_time, bool use_real_time, 
                loss_model_param_t loss_model_params);
        void run();
        std::vector<mobile_node_t> mobile_nodes;
        void report(std::ostream &);
        void update_mobility_model(std::vector<mobile_node_t> mobile_nodes);
        std::vector<neighborhood_t> get_hop_info();
        int total_time;
        NodeContainer backbone;
        NodeContainer stas;

        int getBackboneId(Ipv4Address dest);
        

    private:
        uint32_t n_backbone;
        NodeContainer aps;
        NetDeviceContainer devices;
        std::vector<mobile_node_t> ue_nodes;
        loss_model_param_t loss_model_params;

        Ipv4InterfaceContainer interfaces_bb;
        Ipv4InterfaceContainer interfaces_sta;
        Ipv4InterfaceContainer interfaces_ap;
        InternetStackHelper internet;
        Ipv4AddressHelper address;
        
        AodvHelper aodv_h;
        OlsrHelper olsr_h;

        MobilityHelper mobility;
        bool pcap, print_routes, netanim, verbose;
        int sim_time;
        bool use_real_time;
        void create_backbone_nodes();
        void create_backbone_devices();
        void install_ping_applications();
        void install_inet_stack();
        void create_mobility_model();
        std::thread* simulator;
        void create_sta_nodes(std::vector<mobile_node_t> ue_nodes);
        void create_ap_devices();
        void install_scen1();
        void install_scen2();
        void install_scen3();
        
        //experiments


};
