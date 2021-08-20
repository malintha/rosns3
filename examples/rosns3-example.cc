#include "ns3/rosns3-helper.h"
#include <iostream>
#include "messages/states_generated.h"
#include "comm_model.h"
#include "messages/neighborhoods_generated.h"

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

        std::vector<neighborhood_t> neighborhoods = model->get_hop_info();

        // build the serializable neighborhood object
        flatbuffers::FlatBufferBuilder builder;
        std::vector<flatbuffers::Offset<Neighborhood>> neighborhood_vec;

        for (int i = 0; i < neighborhoods.size(); i++)
        {
        neighborhood_t neighborhood = neighborhoods[i];
          const std::vector<int> neighborhood_temp = neighborhood.neighbors;
          auto neighborhood_fb = CreateNeighborhoodDirect(builder, i, &neighborhood_temp);
          neighborhood_vec.push_back(neighborhood_fb);
        }
        auto neighborhoods_fb = builder.CreateVector(neighborhood_vec);
        auto neighborhoods_s = CreateNeighborhoods(builder, neighborhoods_fb);
        builder.Finish(neighborhoods_s);
        // send the neighborhoods to udp client
        uint8_t* data = builder.GetBufferPointer();
        uint32_t data_size = builder.GetSize();
        server.send_data(data, data_size);

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
