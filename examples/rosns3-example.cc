#include "ns3/rosns3-helper.h"
#include <iostream>
#include "messages/states_generated.h"
#include "comm_model.h"
#include "messages/neighborhoods_generated.h"
#include "messages/network_routing_generated.h"
#include "ns3/olsr-routing-protocol.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/constant-position-mobility-model.h"
#include <fstream>

using namespace ns3;

// commands:
// NS_LOG="ROSNS3Server:ROSNS3Example:ROSNS3Model" ./waf --run rosns3-example --vis
// ./client
// experiment codes:
// tshark -T text -r ap--4-0.pcap -Y "udp && ip.dst == 10.0.0.6" > 4-0.txt
// wc -l 4-0.txt | awk '{ print $1 }'

int main(int argc, char *argv[])
{

  // Setting all these values & Friis parameters from command line at the initialization.
  
  float path_loss_exponent = 2.4;
  float reference_loss = 46;
  float transmission_power = 16.0206;
  float fading_mean = 0;
  float fading_var = 32;

  const uint32_t server_port = 28500;

  CommandLine cmd;

  cmd.AddValue ("exp", "Path loss exponent", path_loss_exponent);
  cmd.AddValue ("ref_loss", "Reference power loss", reference_loss);
  cmd.AddValue ("tx_power", "Transmission power", transmission_power);
  cmd.AddValue ("mean", "Fading mean", fading_mean);
  cmd.AddValue ("var", "Fading variance", fading_var);

  cmd.Parse(argc, argv);
  NS_LOG_COMPONENT_DEFINE("ROSNS3Example");

  std::cout<<"Initializing rosns3-server on port: "<<server_port<<std::endl;
  std::cout<<"Using Friss propagation loss model with:  \n" <<
             "\t Path loss exponent: "<<path_loss_exponent <<"\n" <<
             "\t Reference power loss: "<<reference_loss <<"dBmW \n"<< 
             "\t Tranmission power: "<<transmission_power <<"dBmW \n"<<
             "\t Fading mean: "<<fading_mean << "\n"<<
             "\t Fading variance: "<<fading_var
             <<std::endl;

  loss_model_param_t loss_model_params {.path_loss_exponent = path_loss_exponent, 
                                        . reference_loss = reference_loss,
                                        .transmission_power = transmission_power,
                                        .fading_mean = fading_mean,
                                        .fading_var = fading_var};

  // create the propagation loss model for obtaining the RSS between UAV/UE nodes
  Ptr<PropagationLossModel> log_loss = CreateObject<LogDistancePropagationLossModel> ();
  log_loss->SetAttribute("Exponent", ns3::DoubleValue(path_loss_exponent));
  log_loss->SetAttribute("ReferenceLoss", ns3::DoubleValue(reference_loss));
  Ptr<PropagationLossModel> fading = CreateObject<RandomPropagationLossModel> ();
  const Ptr<NormalRandomVariable> nrv = CreateObject<NormalRandomVariable> ();
  nrv->SetAttribute ("Mean", DoubleValue (fading_mean));
  nrv->SetAttribute ("Variance", DoubleValue (fading_var));
  fading->SetAttribute("Variable", ns3::PointerValue(nrv));
  log_loss->SetNext(fading);
  Ptr<ConstantPositionMobilityModel> a = CreateObject<ConstantPositionMobilityModel> ();
  Ptr<ConstantPositionMobilityModel> b = CreateObject<ConstantPositionMobilityModel> ();
  // double txPowerDbm = (double)transmission_power; // dBm

// write the values to a text file
  
  // std::stringstream ss;
  // if(isdynamic) {
  //   ss << std::to_string(n_backbones)<<"_dynamic.txt";
  // }
  // else {
  //   ss << std::to_string(n_backbones)<<"_static.txt";
  // }
  // std::string filename = ss.str();
  // std::ofstream file_out;
  // file_out.open(filename, std::ios_base::app);

  ROSNS3Server server(server_port);

  bool use_real_time = false;
  bool sim_start = false;
  bool log_rss = true;
  server.start();
  CoModel *model;
  uint64_t it = 0;
  while (server.server_running())
  {
    if (server.data_ready())
    {
      it++;
      recvdata_t data = server.get_data();
      int sim_time = 10;
      ssize_t n_bytes = data.n_bytes;
      char *buffer = data.buffer;
      char agent_data[n_bytes];
      std::memcpy(agent_data, buffer, sizeof(agent_data));
      // NS_LOG_INFO("Recieved: "<< n_bytes << " " << data.timestamp);

      // here goes the deserialization
      auto swarm = GetSwarm(agent_data);
      auto agents = swarm->agents();
      int backbone_nodes = swarm->backbone();
      std::vector<mobile_node_t> mobile_nodes;
      // ns3::RngSeedManager::SetRun(it);

      for (unsigned int i = 0; i < agents->size(); i++)
      {
        const Vec3 *v = agents->Get(i)->position();
        int id = agents->Get(i)->id();
        ns3::Vector pos(v->x(), v->y(), v->z());
        mobile_node_t node = {.position = pos, .id = id};
        mobile_nodes.push_back(node);
      }

      // create the comm model and let the simulation run
      if (!sim_start)
      {
        model = new CoModel(mobile_nodes, backbone_nodes, sim_time, use_real_time, loss_model_params);
        NS_LOG_INFO("Created CoModel");

        model->run();
        sim_start = true;
      }
      else
      {
        model->update_mobility_model(mobile_nodes);
      }
      NS_LOG_INFO("sim_time : " << Simulator::Now().GetSeconds()<<" total_time: "<<model->total_time);

      if (Simulator::Now().GetSeconds() > model->total_time - 1)
      {
        
        // TODO: get the RSS values between the neighboring nodes

        // NodeContainer sta_ = model->stas;
        // float tot_rss = 0;
        // for(int i=0; i< sta_.GetN(); i++) {
        //   Ptr<Node> sta_node = sta_.Get(i);

        //   Ptr<MobilityModel> mob = sta_node->GetObject<MobilityModel>();
        //   Vector pos = mob->GetPosition();
        //   int idx_closest = utils::get_closest_uav(pos, model->backbone);
        //   Ptr<Node> closest_uav_node = model->backbone.Get(idx_closest);
        //   Ptr<MobilityModel> mob_uav = closest_uav_node->GetObject<MobilityModel>();
        //   Vector uav_pos = mob_uav->GetPosition();
        //   // double rss = utils::get_rss(pos, model->backbone, log_loss);
        //   a->SetPosition(pos);
        //   b->SetPosition(uav_pos);
        //   double rss = log_loss->CalcRxPower(txPowerDbm, a, b);
        //   std::cout<<"id: "<<i<<" sta pos: "<<pos <<" uav: "<<uav_pos<<" dis: "<<CalculateDistance(pos,uav_pos)<<std::endl;
        //   tot_rss += rss;
        // }
        // file_out <<Simulator::Now().GetSeconds()<<" , "<<tot_rss/sta_.GetN()<<std::endl;

// TODO: uncomment this to get rss between neighboring UAVs
// this should be after getting the neighborhoods of each robot.

        // get the RSS values of each uav nodes
        // NodeContainer bb_ = model->backbone;
        // for(int i=0; i< bb_.GetN(); i++) {
        //   Ptr<Node> bb_node = bb_.Get(i);

        //   Ptr<MobilityModel> mob = bb_node->GetObject<MobilityModel>();
          
        //   Vector pos = mob->GetPosition();
        //   int idx_closest = utils::get_closest_uav(pos, model->backbone);
        //   Ptr<Node> closest_uav_node = model->backbone.Get(idx_closest);
        //   Ptr<MobilityModel> mob_uav = closest_uav_node->GetObject<MobilityModel>();
        //   Vector uav_pos = mob_uav->GetPosition();
        //   // double rss = utils::get_rss(pos, model->backbone, log_loss);
        //   a->SetPosition(pos);
        //   b->SetPosition(uav_pos);
        //   double rss = log_loss->CalcRxPower(txPowerDbm, a, b);
        //   // std::cout<<"id: "<<i<<" sta pos: "<<pos <<" uav: "<<uav_pos<<" dis: "<<CalculateDistance(pos,uav_pos)<<std::endl;
        //   std::cout <<Simulator::Now().GetSeconds()<<" "<<" "<<i<<" "<<rss<<std::endl;

        // }
        // std::cout<<"Time: "<<Simulator::Now().GetSeconds()<<std::endl;

        NS_LOG_DEBUG("Getting updated routing tables at : " << Simulator::Now().GetSeconds());

        {
          // build network routing object
          flatbuffers::FlatBufferBuilder builder;
          std::vector<flatbuffers::Offset<NetworkNode>> swarm_network;
          NodeContainer backbone_ = model->backbone;

          for (uint32_t i = 0; i < backbone_.GetN(); i++)
          {
            Ptr<Node> node = backbone_.Get(i);
            Ipv4Address source = node->GetObject<Ipv4>()->GetAddress(2, 0).GetLocal();

            Ptr<olsr::RoutingProtocol> rp = node->GetObject<olsr::RoutingProtocol>();
            std::vector<olsr::RoutingTableEntry> table = rp->GetRoutingTableEntries();
            
            // NS_LOG_DEBUG("source: "<<i);
            // for(auto &e :table)
            //   NS_LOG_DEBUG("dest: "<<e.destAddr<<" hops: "<<e.distance);


            // for node routing table
            std::vector<flatbuffers::Offset<Entree>> table_rt;

            // select backbone node entrees from olsr table and create corresponding fb routing table entry
            for (uint32_t j = 0; j < table.size(); j++)
            {
              olsr::RoutingTableEntry entree = table[j];
              Ipv4Address dest = entree.destAddr;
              // NS_LOG_DEBUG("source: "<< i<<" table len: "<<table.size()<<" dest: "<<dest);
              // if(dest.IsEqual(source)) {
              const int dest_id = model->getBackboneId(dest);
              if (dest_id != -1)
              {
                const int dist = entree.distance;
                auto entry_rt = CreateEntree(builder, dest_id, entree.distance);
                table_rt.push_back(entry_rt);
                // NS_LOG_DEBUG("source: "<< i<<" destination: "<<dest_id<<" dist: "<<dist);
              }
              // }
            }
            // create fb network node with routing table and node id
            auto routingtable_fb = builder.CreateVector(table_rt);
            auto network_node = CreateNetworkNode(builder, i, routingtable_fb);
            swarm_network.push_back(network_node);
          }
          auto swarm_network_fb = builder.CreateVector(swarm_network);
          auto swarmnetwork = CreateSwarmNetwork(builder, swarm_network_fb);
          builder.Finish(swarmnetwork);

          NS_LOG_INFO("swarmnetwork: " << swarm_network.size());
          // send the neighborhoods to udp client
          uint8_t *data = builder.GetBufferPointer();
          uint32_t data_size = builder.GetSize();
          server.send_data(data, data_size);
        }

        // {
        // std::vector<neighborhood_t> neighborhoods = model->get_hop_info();
        // build the serializable neighborhood object
        // flatbuffers::FlatBufferBuilder builder;
        // std::vector<flatbuffers::Offset<Neighborhood>> neighborhood_vec;

        // for (int i = 0; i < neighborhoods.size(); i++)
        // {
        // neighborhood_t neighborhood = neighborhoods[i];
        //   const std::vector<int> neighbors = neighborhood.neighbors;
        //   auto neighborhood_fb = CreateNeighborhoodDirect(builder, i, &neighbors);
        //   neighborhood_vec.push_back(neighborhood_fb);
        // }
        // auto neighborhoods_fb = builder.CreateVector(neighborhood_vec);
        // auto neighborhoods_s = CreateNeighborhoods(builder, neighborhoods_fb);
        // builder.Finish(neighborhoods_s);
        // send the neighborhoods to udp client
        // uint8_t* data = builder.GetBufferPointer();
        // uint32_t data_size = builder.GetSize();
        // server.send_data(data, data_size);
        // }
      }
    }

    // else {
    //   NS_LOG_INFO("Waiting for data...");
    // }
    sleep(1);
  }

  sleep(5);
  NS_LOG_INFO("killing server");

  server.kill();
  return 0;
}
