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

NS_LOG_COMPONENT_DEFINE ("ROSNS3Model");

CoModel::CoModel(std::vector<mobile_node_t> mobile_nodes, int backbone_nodes, int sim_time, bool use_real_time):
                mobile_nodes(mobile_nodes),sim_time(sim_time), use_real_time(use_real_time) {
    pcap = true;
    print_routes = true;
    netanim = false;
    verbose = false;
    this->n_backbone = backbone_nodes;
    total_time = sim_time;
    if (use_real_time) {
        GlobalValue::Bind ("SimulatorImplementationType", StringValue (
                            "ns3::RealtimeSimulatorImpl"));
    }

    ue_nodes = utils::get_ue(mobile_nodes, backbone_nodes);
    NS_LOG_DEBUG("Backbone nodes: "<< backbone_nodes << " mobile nodes: "<< mobile_nodes.size());

    // for(int i=0;i<ue_nodes.size();i++)
    //     NS_LOG_DEBUG("UE Locations: "<< "id: "<<i << " pose: "<< ue_nodes[i].position);

    create_backbone_devices();
    create_mobility_model();
    install_inet_stack();
    create_sta_nodes(ue_nodes);
    install_ping_applications();

    // install_scen1();
};

std::vector<neighborhood_t> CoModel::get_hop_info(){
    int hop_length = 1;
    std::vector<neighborhood_t> neighborhoods;
    
    ns3::Ipv4GlobalRoutingHelper::PopulateRoutingTables();
    
    for (uint32_t i = 0; i < backbone.GetN(); i++)
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
            Ipv4Address dest = entree.destAddr;

                // reverse search whose IP address is this in the backbone list
                for(uint32_t k=0; k<backbone.GetN(); k++) {
                    Ptr<Node> cand = backbone.Get(k);
                    Ipv4Address cand_add = cand->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
                    if(dest.IsEqual(cand_add)) {
                        neighbors.push_back(cand->GetId());
                        // NS_LOG_DEBUG( "id: "<< i <<" neighbor: "<< cand->GetId());
                    }
                }
            }
        }
        neighborhood_t neighborhood(i, neighbors);
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
        olsr_h.PrintRoutingTableAllAt (Seconds (sim_time-1), routingStream);
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

// update the mobility model of backbone nodes
void CoModel::update_mobility_model(std::vector<mobile_node_t> mobile_nodes) {
    this->mobile_nodes = mobile_nodes;
    for (uint32_t i = 0; i<n_backbone;i++) {
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

// create mobility model for backbone nodes
void CoModel::create_mobility_model() {
    // add constant mobility model
    Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator>();
    for (uint32_t i = 0;i<n_backbone;i++) {
        mobile_node_t mobile_node = mobile_nodes[i];
        positionAlloc->Add(mobile_node.position);
    }
    mobility.SetPositionAllocator(positionAlloc);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (backbone);
    NS_LOG_DEBUG("Created " << n_backbone << "ground nodes.");
};

void CoModel::report(std::ostream &) {};

void CoModel::install_inet_stack() {
    // you can configure AODV attributes here using aodv.Set(name, value)
    
    internet.SetRoutingHelper (olsr_h);
    internet.Install (backbone);

    address.SetBase ("172.16.0.0", "255.255.255.0");
    interfaces_bb = address.Assign (devices);
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
    // AsciiTraceHelper ascii;
    // wifiPhy.EnableAsciiAll(ascii.CreateFileStream ("trace.tr"));
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
    // ap_devices.Get(0)->Tra
    NS_LOG_INFO("setup infra devices");
    // add ipv4 to infra nodes

    NetDeviceContainer infraDevices (ap_devices, sta_devices);
    internet.Install(stas);

    // Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.255.255.0");
    interfaces_ap = address.Assign(infraDevices);
    
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

    // enable packet capturing for sta nodes
    if (pcap)
    {
        NS_LOG_DEBUG("Enabled packet capturing.");
        // wifiPhy.EnablePcapAll (std::string ("ap-"));
        for (int i=0;i<sta_devices.GetN(); i++)
            wifiPhy.EnablePcap("sta", sta_devices.Get (i), false);
    }

}

// todo: add error model to the channel
void CoModel::create_backbone_devices() {
    backbone.Create(n_backbone);
    for (uint32_t i = 0;i<n_backbone;i++) {
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
void CoModel::install_ping_applications() {
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

    NS_LOG_DEBUG("Installed UDP applications.");

};

// experiments:
// establish udp between the farthest and most centered sta nodes
void CoModel::install_scen1() {
    // install udp between the farthest nodes
    uint16_t port = 9;
    int end_buff = 1;
    int start = 1;
    
    Config::SetDefault ("ns3::OnOffApplication::PacketSize", StringValue ("1472"));
    Config::SetDefault ("ns3::OnOffApplication::DataRate", StringValue ("100Mb/s"));

    uint n_ue = stas.GetN();
    // for(uint i=0; i<n_ue/2; i++) {
        // uint dest_id = n_ue/2 + i;
        Ptr<Node> source1 = stas.Get (0);
        Ptr<Node> destination1 = stas.Get (2);
        Ipv4Address source_add1 = source1->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
        Ipv4Address dest_add1 = destination1->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();
        OnOffHelper onoff ("ns3::UdpSocketFactory", Address (InetSocketAddress (dest_add1, port)));
        ApplicationContainer udp_apps;
        udp_apps.Add(onoff.Install (source1));
        udp_apps.Start(Seconds(start));
        udp_apps.Stop(Seconds(sim_time - end_buff));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (Ipv4Address::GetAny (), port));
        udp_apps.Add(sink.Install(destination1));
        udp_apps.Start(Seconds(start));
        udp_apps.Stop(Seconds(sim_time - end_buff));

        // Ptr<Node> source2 = stas.Get (1);

        // Ptr<Node> destination2 = stas.Get (2);

        // Ipv4Address source_add2 = source2->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();

        // Ipv4Address dest_add2 = destination2->GetObject<Ipv4> ()->GetAddress (1, 0).GetLocal ();

        // // install apps
       

        // onoff = ("ns3::UdpSocketFactory", Address (InetSocketAddress (dest_add2, port)));

        // udp_apps.Add(onoff.Install (source));

        
        // udp_apps = sink.Install(destination);
        // udp_apps.Start(Seconds(start));
        // udp_apps.Stop(Seconds(sim_time - end_buff));
        
        // NS_LOG_DEBUG("Ping source: "<< i+n_backbone <<" "<< source_add << " dest: "<<dest_id+n_backbone <<" "<<dest_add);
    // }

    NS_LOG_DEBUG("Install UDP applications");

}

// communication between the farthest unique links
void CoModel::install_scen2() {

}

// communication between the closest nodes
void CoModel::install_scen3() {

}



