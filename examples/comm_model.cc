#include "comm_model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/nstime.h"
#include "ns3/ssid.h"
#include <fstream>
#include <string>
#include "ns3/applications-module.h"
#include "ns3/on-off-helper.h"
#include "ns3/netanim-module.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/aodv-rtable.h"
#include <sstream>
#include "ns3/aodv-routing-protocol.h"
#include "ns3/olsr-routing-protocol.h"
// #include "utils.h"

// #include "ns3/udp-echo-helper.h"


NS_LOG_COMPONENT_DEFINE ("ROSNS3Model");

CoModel::CoModel(std::vector<mobile_node_t> mobile_nodes, int sim_time, bool use_real_time):
                mobile_nodes(mobile_nodes),sim_time(sim_time), use_real_time(use_real_time) {
    pcap = false;
    print_routes = true;
    netanim = false;
    verbose = false;
    n_nodes = mobile_nodes.size();
    total_time = sim_time;
    if (use_real_time) {
        GlobalValue::Bind ("SimulatorImplementationType", StringValue (
                            "ns3::RealtimeSimulatorImpl"));
    }
    // init roi_params
    Vector2d roi_means(0,0);
    Vector2d roi_vars(20,20);

    ue_nodes = utils::get_ue(roi_means, roi_vars);
    for (uint i=0;i<ue_nodes.size(); i++) {
        NS_LOG_DEBUG("ue: "<<ue_nodes[i].id<<" "<<ue_nodes[i].position << " "<< ue_nodes[i].position.GetLength());
    }

    // create_backbone_nodes();
    create_backbone_devices();
    create_mobility_model();
    install_inet_stack();

    create_sta_nodes(ue_nodes);
    // NS_LOG_INFO("Created sta nodes");

    // install_applications();
    install_scen1();
};

std::vector<neighborhood_t> CoModel::get_hop_info(){
    int hop_length = 1;
    std::vector<neighborhood_t> neighborhoods;

    // Ptr<OutputStreamWrapper> ss = Create<OutputStreamWrapper> ("aodv.routes1", std::ios::out);
    for (uint32_t i = 0; i < n_nodes; i++)
    {
        Ptr<Node> node = backbone.Get(i);
        // Ptr<aodv::RoutingProtocol> ipv4 = node->GetObject<aodv::RoutingProtocol> ();
        Ptr<olsr::RoutingProtocol> rp = node->GetObject<olsr::RoutingProtocol> ();
        // std::cout << "table length: "<<std::endl; 
        std::vector<olsr::RoutingTableEntry> table = rp->GetRoutingTableEntries();
        std::vector<int> neighbors;
        for(uint32_t j=0;j<table.size(); j++) {
            olsr::RoutingTableEntry entree = table[j];
            if(entree.distance <= hop_length) {
                neighbors.push_back(j);
            }
            // std::cout << "entree: "<<table[j].destAddr << " " << table[j].distance<<std::endl; 
        }
        neighborhood_t neighborhood(i, neighbors);
        // neighborhood.id = i;
        // neighborhood.neighbors = neighbors;
        neighborhoods.push_back(neighborhood);
    }
    return neighborhoods;
}

void CoModel::run() {
    auto simulator_t = [this]() {
    if(this->netanim) {
            AnimationInterface anim ("ros-ns3.xml");
    }
    if (print_routes)
    {
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
        aodv_h.PrintRoutingTableAllAt (Seconds (sim_time-1), routingStream);
    }
        Simulator::Stop (Seconds (sim_time));        
        Simulator::Run ();
        NS_LOG_DEBUG("Finished simulation. Time: " << Simulator::Now().GetSeconds());

        // report(std::cout);
        // Simulator::Destroy();
    };
    if(use_real_time) {
        NS_LOG_DEBUG("Simulation started in a new thread.");
        this->simulator = new std::thread(simulator_t);
    }
    else {
        NS_LOG_DEBUG("Simulation started.");
        simulator_t();
    }
};

void CoModel::update_mobility_model(std::vector<mobile_node_t> mobile_nodes) {
    this->mobile_nodes = mobile_nodes;
    for (uint32_t i = 0; i<n_nodes;i++) {
        Ptr<Node> node = backbone.Get (i);
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
        mob->SetPosition(mobile_nodes[i].position);
    }
    total_time += sim_time;
    NS_LOG_DEBUG("Simulating with the updated mobility model. Total time: "<< total_time);
    
    if (!use_real_time) {
        run();
    }
};

void CoModel::create_mobility_model() {
    // add constant mobility model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0;i<n_nodes;i++) {
        mobile_node_t mobile_node = mobile_nodes[i];
        positionAlloc->Add(mobile_node.position);
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (backbone);
    NS_LOG_DEBUG("Created " << n_nodes << "ground nodes.");
};

void CoModel::report(std::ostream &) {};

void CoModel::create_backbone_nodes() {

};

void CoModel::install_inet_stack() {
    // you can configure AODV attributes here using aodv.Set(name, value)
    
    internet.SetRoutingHelper (olsr_h);
    internet.Install (backbone);

    address.SetBase ("172.16.0.0", "255.255.255.0");
    interfaces_bb = address.Assign (devices);

    // if (print_routes)
    // {
    //     Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
    //     aodv.PrintRoutingTableAllAt (Seconds (sim_time-1), routingStream);
    // }
    NS_LOG_DEBUG("Installed AODV internet stack.");

};


// create the sta nodes and the ap nodes
void CoModel::create_sta_nodes(std::vector<mobile_node_t> ue_nodes) {
    stas.Create(ue_nodes.size());
    WifiHelper wifi_infra;

    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();

    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", ns3::DoubleValue(2.52),
                                    "ReferenceLoss", ns3::DoubleValue(-53));
    wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel","m0",ns3::DoubleValue(1),
                                    "m1",ns3::DoubleValue(1),"m2",ns3::DoubleValue(1));
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");

    wifiPhy.SetChannel (wifiChannel.Create ());
    Ssid ssid = Ssid ("wifi-infra");
    wifi_infra.SetRemoteStationManager ("ns3::ArfWifiManager");
    WifiMacHelper mac_infra;

    // setup stas
    mac_infra.SetType("ns3::StaWifiMac", "Ssid", SsidValue (ssid));
    NetDeviceContainer sta_devices = wifi_infra.Install (wifiPhy, mac_infra, stas);

    // setup aps
    aps = backbone;
    mac_infra.SetType("ns3::ApWifiMac", "Ssid", SsidValue (ssid));
    NetDeviceContainer ap_devices = wifi_infra.Install(wifiPhy, mac_infra, aps);
    
    NS_LOG_INFO("setup infra devices");
    // add ipv4 to infra nodes

    NetDeviceContainer infraDevices (ap_devices, sta_devices);
    internet.Install(stas);

    // Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.255.255.0");
    // stack.Install(aps);
    interfaces_ap = address.Assign(infraDevices);
    // interfaces_sta = address.Assign(sta_devices);

    // constant mobility for sta nodes
    MobilityHelper mobility_sta;

    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (unsigned int i = 0;i<ue_nodes.size();i++) {
        mobile_node_t ue_node = ue_nodes[i];
        positionAlloc->Add(ue_node.position);
    }
    NS_LOG_INFO("Setting STA mobility model");
    mobility_sta.SetPositionAllocator(positionAlloc);
    mobility_sta.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    // mobility_sta.SetMobilityModel ("ns3::RandomDirection2dMobilityModel",
    //                                "Bounds", RectangleValue (Rectangle (-30, 30, -30, 30)),
    //                                "Speed", StringValue ("ns3::ConstantRandomVariable[Constant=1]"),
    //                                "Pause", StringValue ("ns3::ConstantRandomVariable[Constant=0.4]"));

    mobility_sta.Install(stas);
    NS_LOG_DEBUG("Created STA and AP nodes.");

    if (pcap)
    {
        NS_LOG_DEBUG("Enabled packet capturing.");
        wifiPhy.EnablePcapAll (std::string ("ap-"));
    }

}

// todo: add error model to the channel
void CoModel::create_backbone_devices() {
    backbone.Create(n_nodes);
    for (uint32_t i = 0;i<n_nodes;i++) {
        std::ostringstream os;
        os << "backbone-" << i+1;
        Names::Add (os.str (), backbone.Get (i));
    }

    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");

    // yanswifiPhy by defaul implements 802.11a. the maximum ofdm rate for 802.11a is 54 which is used here.
    //todo: how to set transmission power? is that ok to be set in the loss model???
    //tune Nakagami to zero mean rv
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();

    wifiChannel.AddPropagationLoss("ns3::LogDistancePropagationLossModel", "Exponent", ns3::DoubleValue(2.52),
                                    "ReferenceLoss", ns3::DoubleValue(-53));
    wifiChannel.AddPropagationLoss("ns3::NakagamiPropagationLossModel","m0",ns3::DoubleValue(1),
                                    "m1",ns3::DoubleValue(1),"m2",ns3::DoubleValue(1));
    wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
    wifiPhy.SetChannel (wifiChannel.Create ());
    

    /**
     * https://www.nsnam.org/docs/release/3.21/doxygen/classns3_1_1_constant_rate_wifi_manager.html
     * 
     * ConstantRateWifiManager: This class uses always the same transmission rate for 
     * every packet sent. 
     * DataMode: The transmission mode to use for every data packet transmission 
     * (Initial value: OfdmRate6Mbps)
     * RtsCtsThreshold: If the size of the data packet + LLC header + MAC header + FCS 
     * trailer is bigger than this value, we use an RTS/CTS handshake before sending the 
     * data, as per IEEE Std. 802.11-2012, Section 9.3.5. This value will not have any 
     * ffect on some rate control algorithms.
     * 
     * **/
    WifiHelper wifi;
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", 
                    StringValue ("OfdmRate54Mbps"), "RtsCtsThreshold", UintegerValue (0));
    devices = wifi.Install (wifiPhy, wifiMac, backbone); 

    if (pcap)
    {
        NS_LOG_DEBUG("Enabled packet capturing.");
        wifiPhy.EnablePcapAll (std::string ("rosns3"));
    }
    NS_LOG_DEBUG("Installed wifi devices.");

};

// install applications on sta nodes
void CoModel::install_applications() {
    // address of the last sta
    // NS_LOG_DEBUG("Creating UDP Applications");
    
    // Config::SetDefault ("ns3::OnOffApplication::PacketSize", StringValue ("1472"));
    // Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("1000kb/s"));
    
    // uint16_t port = 9;
    // Ptr<Node> appSource = stas.Get(0); //10.0.0.3
    // Ptr<Node> appSink = stas.Get(4); //10.0.0.(3+4)
    // Ipv4Address remoteAddr = appSink->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
    // OnOffHelper onoff ("ns3::UdpSocketFactory",
    //                  Address (InetSocketAddress (remoteAddr, port)));  

    // ApplicationContainer apps = onoff.Install (appSource);
    // apps.Start (Seconds (1));
    // apps.Stop (Seconds (sim_time) - Seconds (1));

    // PacketSinkHelper sink ("ns3::UdpSocketFactory",
    //                      InetSocketAddress (Ipv4Address::GetAny (), port));
    // apps = sink.Install (appSink);
    // apps.Start (Seconds (1));


    Ipv4Address remoteAddr("10.0.0.10");
    V4PingHelper ping(remoteAddr);
    // V4PingHelper ping(interfaces_bb.GetAddress (2));

    ping.SetAttribute("Verbose", BooleanValue(verbose));
    const Time t = Seconds(1);
    ping.SetAttribute("Interval", TimeValue(t));

    // // install ping app on source sta
    ApplicationContainer apps;
    Ptr<Node> source = NodeList::GetNode (n_nodes);

    apps.Add(ping.Install(source));
    Ipv4Address sourceAdd = source->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
    NS_LOG_DEBUG("Ping source: "<< sourceAdd);

    apps.Start(Seconds(3));
    apps.Stop(Seconds(sim_time) - Seconds(0.5));

    NS_LOG_DEBUG("Installed UDP applications.");

};

// establish ping between the farthest and most centered sta nodes
void CoModel::install_scen1() {
    // install ping between the farthest nodes
    ApplicationContainer ping_apps;
    uint n_ue = stas.GetN();
    for(uint i=0; i<n_ue/2; i++) {
        uint dest_id = n_ue/2 + i;
        Ptr<Node> source = stas.Get (i);
        Ipv4Address source_add = source->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
        Ptr<Node> destination = stas.Get (dest_id);
        Ipv4Address dest_add = destination->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();

        V4PingHelper ping(dest_add);
        ping.SetAttribute("Verbose", BooleanValue(verbose));
        const Time t = Seconds(0.5);
        ping.SetAttribute("Interval", TimeValue(t));
        // // install ping app on source sta
        ping_apps.Add(ping.Install(source));
        NS_LOG_DEBUG("Ping source: "<< source_add << " dest: "<<dest_add);

    }
    ping_apps.Start(Seconds(1));
    ping_apps.Stop(Seconds(sim_time) - Seconds(1));

}

// communication between the farthest unique links
void CoModel::install_scen2() {

}

// communication between the closest nodes
void CoModel::install_scen3() {

}



