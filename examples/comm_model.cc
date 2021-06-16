#include "comm_model.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/aodv-module.h"
#include "ns3/nstime.h"

NS_LOG_COMPONENT_DEFINE ("ROSNS3Model");

CoModel::CoModel(std::vector<mobile_node_t> mobile_nodes, int sim_time, bool use_real_time):
                mobile_nodes(mobile_nodes),sim_time(sim_time), use_real_time(use_real_time) {
    pcap = true;
    print_routes = false;
    n_nodes = mobile_nodes.size();

    if (use_real_time) {
        GlobalValue::Bind ("SimulatorImplementationType", StringValue (
                            "ns3::RealtimeSimulatorImpl"));
    }
    create_nodes();
    create_mobility_model();
    create_devices();
    install_inet_stack();
    install_applications();
};

void CoModel::run() {
    auto simulator_t = [this]() {
        Simulator::Stop (Seconds (sim_time));        
        Simulator::Run ();
        NS_LOG_DEBUG("Finished simulation.");
        report(std::cout);
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
        Ptr<Node> node = node_container.Get (i);
        Ptr<MobilityModel> mob = node->GetObject<MobilityModel> ();
        mob->SetPosition(mobile_nodes[i].position);
    }
    NS_LOG_DEBUG("Simulating with the updated the mobility model.");
    
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
    mobility.Install (node_container);
    NS_LOG_DEBUG("Created " << n_nodes << " nodes.");
};

void CoModel::report(std::ostream &) {};

void CoModel::create_nodes() {
    node_container.Create(n_nodes);
    for (uint32_t i = 0;i<n_nodes;i++) {
        std::ostringstream os;
        os << "node-" << i+1;
        Names::Add (os.str (), node_container.Get (i));
    }
};

void CoModel::create_devices() {
    WifiMacHelper wifiMac;
    wifiMac.SetType ("ns3::AdhocWifiMac");
    YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel = YansWifiChannelHelper::Default ();
    wifiPhy.SetChannel (wifiChannel.Create ());
    WifiHelper wifi;

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
    wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", 
                    StringValue ("OfdmRate6Mbps"), "RtsCtsThreshold", UintegerValue (0));
    devices = wifi.Install (wifiPhy, wifiMac, node_container); 

    if (pcap)
    {
        NS_LOG_DEBUG("Enabled packet capturing.");
        wifiPhy.EnablePcapAll (std::string ("rosns3"));
    }
    NS_LOG_DEBUG("Installed wifi devices.");

};

void CoModel::install_applications() {
    V4PingHelper ping (interfaces.GetAddress (n_nodes - 1));
    ping.SetAttribute ("Verbose", BooleanValue (true));
    const Time t = Seconds(1);
    ping.SetAttribute ("Interval", TimeValue(t));

    ApplicationContainer p = ping.Install (node_container.Get (0));
    ApplicationContainer p1 = ping.Install (node_container.Get (1));

    p.Start (Seconds (0));
    p.Stop (Seconds (sim_time) - Seconds (0.001));

    p1.Start (Seconds (0));
    p1.Stop (Seconds (sim_time) - Seconds (0.001));
    NS_LOG_DEBUG("Installed ping applications.");

};

void CoModel::install_inet_stack() {
    AodvHelper aodv;
    // you can configure AODV attributes here using aodv.Set(name, value)
    InternetStackHelper stack;
    stack.SetRoutingHelper (aodv); // has effect on the next Install ()
    stack.Install (node_container);
    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.0.0.0");
    interfaces = address.Assign (devices);

    if (print_routes)
    {
        Ptr<OutputStreamWrapper> routingStream = Create<OutputStreamWrapper> ("aodv.routes", std::ios::out);
        aodv.PrintRoutingTableAllAt (Seconds (sim_time-1), routingStream);
    }
    NS_LOG_DEBUG("Installed AODV internet stack.");

};

