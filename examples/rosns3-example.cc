#include "ns3/rosns3-helper.h"
#include <iostream>
#include "messages/states_generated.h"
#include "comm_model.h"

using namespace ns3;

// commands:
// NS_LOG="ROSNS3Server:ROSNS3Example:ROSNS3Model" ./waf --run rosns3-example
// ./client

int main (int argc, char *argv[])
{
  NS_LOG_COMPONENT_DEFINE ("ROSNS3Example");

  ROSNS3Server server(28500);
  bool use_real_time = false;
  bool sim_start = false;
  server.start();
  CoModel* model;

  while(server.get_server_status()) {
    if(server.data_ready()) {
      recvdata_t data = server.get_data();
      int sim_time = 10;
      ssize_t n_bytes = data.n_bytes;
      char* buffer = data.buffer;
      char agent_data[n_bytes];
      std::memcpy(agent_data, buffer, sizeof(agent_data));
      // NS_LOG_INFO("Recieved: "<< n_bytes << " " << data.timestamp);

      // here goes the deserialization
      auto swarm = GetSwarm(agent_data);
      auto agents = swarm->agents();
      std::vector<mobile_node_t> mobile_nodes;
      for (unsigned int i=0;i<agents->size();i++) {
        const Vec3 *v = agents->Get(i)->position();
        int id = agents->Get(i)->id();
        Vector pos(v->x(),v->y(),v->z());
        mobile_node_t node = {.position= pos, .id=id};
        mobile_nodes.push_back(node);
      }

      // create the comm model and let the simulation run
      if (!sim_start) {
        model = new CoModel(mobile_nodes, sim_time, use_real_time);
        NS_LOG_INFO("Created CoModel");            

        model->run();
        sim_start = true;
      } 
      else {
        model->update_mobility_model(mobile_nodes);
      }
    }

    else {
      NS_LOG_INFO("Waiting for data...");            
    }
    sleep(1);

  }

  sleep(5);
  NS_LOG_INFO("killing server");

  server.kill();
  return 0;
}


