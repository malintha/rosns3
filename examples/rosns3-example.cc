/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "ns3/core-module.h"
#include "ns3/rosns3-helper.h"
#include <iostream>
#include "messages/states_generated.h"

using namespace ns3;

int main (int argc, char *argv[])
{
  bool verbose = true;

  CommandLine cmd;
  cmd.AddValue ("verbose", "Tell application to log if true", verbose);

  ROSNS3Server server(28500);
  
  server.start();
  while(server.get_server_status()) {
    recvdata_t data = server.get_data();
    std::cout << data.n_bytes << std::endl;
    if(data.n_bytes != 0) {
      ssize_t n_bytes = data.n_bytes;
      char* buffer = data.buffer;
      char agent_data[n_bytes];
      std::memcpy(agent_data, buffer, sizeof(agent_data));
      std::cout << "recieved: "<< n_bytes << " " << agent_data << std::endl;

    // here goes the deserialization

    }
    sleep(1);

  }

  sleep(5);
  std::cout << "killing server"<< std::endl;

  server.kill();
  cmd.Parse (argc,argv);
  // Simulator::Run ();
  // std::cout << "The value of x is " << std::endl;

  // Simulator::Destroy ();
  return 0;
}


