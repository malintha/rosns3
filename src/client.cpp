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
#include "std_msgs/Int16MultiArray.h"
#include "std_msgs/MultiArrayDimension.h"
#include "eigen3/Eigen/Dense"

Client::Client(clientutils::params_t params, ros::NodeHandle n) : params(params), n(n)
{
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    client_busy = false;
    set_nodes();

    routing_table_pub = n.advertise<std_msgs::Int16MultiArray>("/routing_table", 10);
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
        utils::Node *node = new utils::Node(i + 1, n, i<params.n_backbone);
        nodes.push_back(node);
    }
    ROS_DEBUG_STREAM("Created " << params.n_robots << " with "<<params.n_backbone << " backbones.");
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
            rosns3_client::Waypoint state = nodes[i]->get_state();
            ROS_INFO_STREAM(i<<" "<<state.position.x<< " " << state.position.y << " " << state.position.z);
            auto pos = Vec3(state.position.x, state.position.y, state.position.z);
            auto id = i;
            auto agent = CreateAgent(builder, &pos, id);
            agents.push_back(agent);
        }
        // std::cout<<std::endl;
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
            routing_tables = this->set_network();
        };
        new std::thread(con);
    }
    else
    {
        ROS_DEBUG_STREAM("Iteration skipping. Client busy");
    }

    publish_routing_table();
}

routing_table_t Client::set_network() {
    routing_table_t routing_table;
    char recvd_data[n_bytes];
    std::memcpy(recvd_data, recv_buffer, sizeof(recvd_data));
    ROS_DEBUG_STREAM("Received " << sizeof(recvd_data) << " bytes.");
    routing_table.clear();

    // receive the router tables as is from server
    auto swarmnetwork = GetSwarmNetwork(recvd_data);
    // auto network_nodes = swarmnetwork->nodes();
    auto networknodes = swarmnetwork->nodes();
    for(int i=0; i<networknodes->size(); i++) {
        std::vector<neighborpair> routing_nodes;
        auto routingtable = networknodes->Get(i)->routingtable();
        for (int j=0;j<routingtable->size(); j++) {
            auto entry = routingtable->Get(j);
            int hops = entry->distance();
            if (hops <= params.hops_k) {
                neighborpair np(entry->destination(), hops);
                routing_nodes.push_back(np);
            }
        }
        // nodes[i]->set_routing_nodes(routing_nodes);
        // set the routing nodes into a multi-dimensional vector
        routing_table.push_back(routing_nodes);
    }
    return routing_table;
}

// create adjacency matrix -> use it to initialize the multi-array
void Client::publish_routing_table() {
    // first create the adjacency matrix
    Eigen::MatrixXi adjacency = Eigen::MatrixXi::Zero(params.n_backbone,params.n_backbone);
    ROS_DEBUG_STREAM("publishing routing table. Routing tables size: "<<routing_tables.size());
    for(int i=0; i<routing_tables.size(); i++) {
        std::vector<neighborpair> table_i = routing_tables[i];
        adjacency(i,i) = 1;
        for(int j=0; j<params.n_backbone; j++) {
            int dis = clientutils::has_value(table_i, j);
            if(dis > 0) {
                adjacency(i,j) = dis;
            }    
        }
    }
    // std::cout << "adjacency mat: "<<adjacency <<std::endl;
// use adjacency matrix to populate the publising message
    ROS_DEBUG_STREAM("\n"<<adjacency);

    std_msgs::Int16MultiArray msg;

    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    msg.layout.dim.push_back(std_msgs::MultiArrayDimension());

    msg.layout.dim[0].size = params.n_backbone;
    msg.layout.dim[0].stride = params.n_backbone*params.n_backbone;
    msg.layout.dim[0].label = "rows";
    
    msg.layout.dim[1].size = params.n_backbone;
    msg.layout.dim[1].stride = params.n_backbone;
    msg.layout.dim[1].label = "columns";
    
    for(int i=0;i<adjacency.size(); i++) {
        msg.data.push_back(adjacency.data()[i]);
    }

    routing_table_pub.publish(msg);
    // ROS_INFO_STREAM("Published routing table for "<<params.n_backbone << " nodes.");

}
