#include "client.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
// #include "neighborhoods_generated.h"
#include <thread>
#include <unistd.h>
#include <iostream>
#include <algorithm>

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

recv_data_t *Client::send_recv_data()
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

    // char buffer[MAXLINE];

    n_bytes = recvfrom(sockfd, recv_buffer, MAXLINE,
                       MSG_WAITALL, (struct sockaddr *)&servaddr,
                       &len);
    recv_data_t routing_data;
    routing_data.n_bytes = n_bytes;

    routing_data.recv_buffer = recv_buffer;
    client_busy = false;
    return &routing_data;
    // close(sockfd);
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
        utils::Node *node = new utils::Node(i + 1, n, params.frequency);
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
            this->set_network();
        };
        new std::thread(con);
    }
    else
    {
        ROS_DEBUG_STREAM("Iteration skipping. Client busy");
    }

    // publish current routing nodes
    for(clientutils::Node* n: nodes) {
        n->publish_routing_nodes();
        ROS_DEBUG_STREAM("Published routing tables.");
    }
}

void Client::set_network() {
    char recvd_data[n_bytes];
    std::memcpy(recvd_data, recv_buffer, sizeof(recvd_data));
    ROS_DEBUG_STREAM("Received " << sizeof(recvd_data) << " bytes.");

    // receive the router tables as is from server
    auto swarmnetwork = GetSwarmNetwork(recvd_data);
    // auto network_nodes = swarmnetwork->nodes();
    this->network = swarmnetwork->nodes();
    for(int i=0; i<nodes.size(); i++) {
        std::vector<int> routing_nodes;
        auto routingtable = network->Get(i)->routingtable();
        for (int j=0;j<routingtable->size(); j++) {
            auto entry = routingtable->Get(j);
            int hops = entry->distance();
            if (hops == params.hops_k) {
                routing_nodes.push_back(entry->destination());
            }
        }
        nodes[i]->set_routing_nodes(routing_nodes);
    }
}

// calculate the average neighborhood hops against distance
void Client::calc_plot_info()
{
    auto network_nodes = this->network;
    // get avg hops between the robots in the network
    std::vector<int> hops;
    std::vector<int> entries;

    for (int i = 0; i < network_nodes->size(); i++)
    {
        int n_id = network_nodes->Get(i)->node();
        auto routingtable = network_nodes->Get(i)->routingtable();
        // ROS_INFO_STREAM("node: " << n_id << " table entries: " << routingtable->size());
        entries.push_back(routingtable->size());

        for (int j = 0; j < routingtable->size(); j++)
        {
            auto entry = routingtable->Get(j);
            if (entry->destination() != i)
            {
                hops.push_back(entry->distance());
            }
        }
        if (routingtable->size() < 7)
        {
            int balance = 7 - routingtable->size();
            for (int j = 0; j < balance; j++)
            {
                hops.push_back(0);
            }
        }
    }
    //get avg distance between nodes
    double avg_dis = clientutils::get_avg_dist(nodes);
    clientutils::write_to_file(hops, avg_dis);
    ROS_INFO_STREAM("avg_dis: "<<avg_dis<<" wrote hops info to file.");

    // for(int n_entrees: entries) {
    //     std::cout << " "<< n_entrees;
    // }
    // std::cout << std::endl;
}