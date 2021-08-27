#include "ns3/rosns3-helper.h"
#include <iostream>
#include "messages/states_generated.h"
#include "comm_model.h"
#include "messages/neighborhoods_generated.h"
#include "messages/network_routing_generated.h"
#include "ns3/olsr-routing-protocol.h"

using namespace ns3;

// commands:
// NS_LOG="ROSNS3Server:ROSNS3Example:ROSNS3Model" ./waf --run rosns3-example --vis
// ./client
// experiment codes: 
// tshark -T text -r ap--4-0.pcap -Y "udp && ip.dst == 10.0.0.6" > 4-0.txt 
// wc -l 4-0.txt | awk '{ print $1 }'

int main(int argc, char *argv[])
{
  CommandLine cmd;
  cmd.Parse(argc, argv);

  NS_LOG_COMPONENT_DEFINE("ROSNS3Example");

  ROSNS3Server server(28500);
  bool use_real_time = false;
  bool sim_start = false;
  server.start();
  CoModel *model;

  while (server.server_running())
  {
    if (server.data_ready())
    {
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
      for (unsigned int i = 0; i < agents->size(); i++)
      {
        const Vec3 *v = agents->Get(i)->position();
        int id = agents->Get(i)->id();
        Vector pos(v->x(), v->y(), v->z());
        mobile_node_t node = {.position = pos, .id = id};
        mobile_nodes.push_back(node);
      }

      // create the comm model and let the simulation run
      if (!sim_start)
      {
        model = new CoModel(mobile_nodes,backbone_nodes, sim_time, use_real_time);
        NS_LOG_INFO("Created CoModel");

        model->run();
        sim_start = true;
      }
      else
      {
        model->update_mobility_model(mobile_nodes);
      }
      if (Simulator::Now().GetSeconds() > model->total_time - 1)
      {
        NS_LOG_INFO("Getting updated routing tables at : " << Simulator::Now().GetSeconds());

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
          // NS_LOG_DEBUG("###: i "<<i<< " ip: "<<source);

          // for node routing table
          std::vector<flatbuffers::Offset<Entree>> table_rt;
          
          // select backbone node entrees from olsr table and create corresponding fb routing table entry
          for (uint32_t j = 0; j < table.size(); j++)
          {
            olsr::RoutingTableEntry entree = table[j];
            Ipv4Address dest = entree.destAddr;
            // NS_LOG_DEBUG("source: "<< i<<" table len: "<<table.size()<<" dest: "<<dest);
            const int dest_id = model->getBackboneId(dest);
            if (dest_id != -1)
              {
                const int dist = entree.distance;
                auto entry_rt = CreateEntree(builder, dest_id, entree.distance);
                table_rt.push_back(entry_rt);
                // NS_LOG_DEBUG("source: "<< i<<" destination: "<<dest_id<<" dist: "<<dist);
              }
          }
          // create fb network node with routing table and node id
          auto routingtable_fb = builder.CreateVector(table_rt);
          auto network_node = CreateNetworkNode(builder, i, routingtable_fb);
          swarm_network.push_back(network_node);
        }
        auto swarm_network_fb = builder.CreateVector(swarm_network);
        auto swarmnetwork = CreateSwarmNetwork(builder,swarm_network_fb);  
        builder.Finish(swarmnetwork);

        NS_LOG_INFO("swarmnetwork: "<<swarm_network.size());
        // send the neighborhoods to udp client
        uint8_t* data = builder.GetBufferPointer();
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
