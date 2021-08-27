#include "client.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "neighborhoods_generated.h"
#include <thread>
#include <unistd.h>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include "network_routing_generated.h"

Client::Client(clientutils::params_t params, ros::NodeHandle n) : params(params), n(n)
{
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    client_busy = false;
    set_nodes();
}

void Client::send_recv_data()
{
    client_busy = true;
    socklen_t len;
    struct sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));

    // Filling server information
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(params.port);
    servaddr.sin_addr.s_addr = INADDR_ANY;

    sendto(sockfd, data, data_size,
           MSG_CONFIRM, (const struct sockaddr *)&servaddr,
           sizeof(servaddr));
    ROS_DEBUG_STREAM("Sent " << data_size << " bytes of data to the NS3 server.");
    ROS_DEBUG_STREAM("Waiting for the server to respond...");

    char buffer[MAXLINE];
    ssize_t n_bytes = recvfrom(sockfd, buffer, MAXLINE,
                               MSG_WAITALL, (struct sockaddr *)&servaddr,
                               &len);

    char agent_data[n_bytes];
    std::memcpy(agent_data, buffer, sizeof(agent_data));
    ROS_DEBUG_STREAM("Received " << sizeof(agent_data) << " bytes.");

    // deserialize recieved data and copy them to global neighborhoods variable
    // this section is for getting neighborlists from the server
    // auto nbrhoods_temp = GetNeighborhoods(agent_data);
    // auto nbrhoods_fb = nbrhoods_temp->neighborhood();
    // neighborhoods.clear();
    // for (int i = 0; i < nbrhoods_fb->size(); i++)
    // {
    //     std::vector<int> nbrs;
    //     int n_id = nbrhoods_fb->Get(i)->id();
    //     const auto neighbors = nbrhoods_fb->Get(i)->neighbors();
    //     for (int j = 0; j < neighbors->size(); j++)
    //     {
    //         auto id = neighbors->Get(j);
    //         nbrs.push_back(id);
    //     }
    //     clientutils::neighborhood_t nbrhood {.id = n_id, .neighbors = nbrs};
    //     this->neighborhoods.push_back(nbrhood);
    // }

// receive the router tables as is from server
    auto swarmnetwork = GetSwarmNetwork(agent_data);
    auto network_nodes = swarmnetwork->nodes();
    for (int i = 0; i < network_nodes->size(); i++)
    {
        int n_id = network_nodes->Get(i)->node();
        auto routingtable = network_nodes->Get(i)->routingtable();
        
        ROS_INFO_STREAM("node: "<<n_id<<" table entries: "<<routingtable->size());
        // for(int j=0;j<routingtable->size(); j++) {
        //     auto entry = routingtable->Get(j);
        //     ROS_INFO_STREAM("    destination: "<<entry->destination()<<" hops: "<<entry->distance());
        // }
                
    }

    ROS_DEBUG_STREAM("Deserialized the routing table data.");

    client_busy = false;

    // close(sockfd);
}

std::vector<utils::neighborhood_t> Client::get_neighborhoods() {
    return this->neighborhoods;
}

void Client::run()
{
    ros::Timer timer = n.createTimer(ros::Duration(1 / params.frequency), &Client::iteration, this);
    ros::spin();
}

void Client::set_nodes()
{
    // utils::nodes_t nodes_;
    // nodes.clear();
    for (int i = 0; i < params.n_robots; i++)
    {
        utils::Node* node = new utils::Node(i+1, n);
        nodes.push_back(node);
    }
    ROS_DEBUG_STREAM("Created " << params.n_robots << " nodes and subscribers.");
}

void Client::iteration(const ros::TimerEvent &e)
{
    //get data from subscribers
    if (!client_busy)
    {
        ROS_DEBUG_STREAM("Iteration performing.");

        flatbuffers::FlatBufferBuilder builder;
        agents_t agents;

        for (int i = 0; i < params.n_robots; i++)
        {
            simulator_utils::Waypoint state = nodes[i]->get_state();
            // ROS_DEBUG_STREAM(state.position.x << state.position.y << state.position.z);
            auto pos = Vec3(state.position.x, state.position.y, state.position.z);
            auto id = i;
            auto agent = CreateAgent(builder, &pos, id);
            agents.push_back(agent);
        }

        //create the swarm object
        auto agents_ = builder.CreateVector(agents);
        auto swarm = CreateSwarm(builder, params.n_backbone, agents_);
        builder.Finish(swarm);
        data = builder.GetBufferPointer();
        data_size = builder.GetSize();

        // send data to the server in a non-blocking call (if the previous call to the
        // server is completed), wait for the response. Write the response to the variable.
        auto con = [this]()
        {
            this->send_recv_data();
            // this->log_info();
        };
        new std::thread(con);
    }
    else
    {
        ROS_DEBUG_STREAM("Iteration skipping. Client busy");
    }

    // publish current neighborhoods to a service

}

void Client::log_info() {
    // calculate the mean/min/max agents and log with avg distance
    double tot_nbrs;
    int min_nbrs = params.n_backbone;
    int max_nbrs = 0;
    for(int i=0;i<neighborhoods.size(); i++) {
        tot_nbrs += neighborhoods[i].neighbors.size();
        if (neighborhoods[i].neighbors.size() < min_nbrs)
            min_nbrs = neighborhoods[i].neighbors.size();
        if (neighborhoods[i].neighbors.size() > max_nbrs)
            max_nbrs = neighborhoods[i].neighbors.size();
    }

    std::vector<double> dist;
    for(int i=0; i<nodes.size(); i++) {
        simulator_utils::Waypoint state_i = nodes[i]->get_state(); 
        Eigen::Vector3d pos_i(state_i.position.x, state_i.position.y, state_i.position.z);
        for(int j=0; j<nodes.size(); j++) {
            if(j==i)
                continue;
            simulator_utils::Waypoint state_j = nodes[j]->get_state(); 
            Eigen::Vector3d pos_j(state_j.position.x, state_j.position.y, state_j.position.z);;
            dist.push_back((pos_i - pos_j).norm());
        }
    }
    double tot_dis;
    for(double d: dist) {
        tot_dis += d;
    }
    ROS_INFO_STREAM(tot_dis/dist.size() << " "<<tot_nbrs/params.n_backbone << " "<< min_nbrs <<" "<<max_nbrs);


}