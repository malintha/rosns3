#include "client.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include "neighborhoods_generated.h"
#include <thread>
#include <unistd.h>
#include <iostream>

Client::Client(clientutils::params_t params, ros::NodeHandle n) : params(params), n(n)
{
    if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0)
    {
        perror("socket creation failed");
        exit(EXIT_FAILURE);
    }

    client_busy = false;
    get_nodes();
}

void Client::send_recv_data()
{
    this->client_busy = true;
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
    // deserialize recieved data
    auto neighborhoods_temp = GetNeighborhoods(agent_data);
    auto neighborhoods = neighborhoods_temp->neighborhood();
    for (int i = 0; i < neighborhoods->size(); i++)
    {
        printf("id: %d \t", neighborhoods->Get(i)->id());
        const auto neighbors = neighborhoods->Get(i)->neighbors();
        for (int j = 0; j < neighbors->size(); j++)
        {
            // auto id = neighbors->Get(j);
            printf("%d ", neighbors->Get(j));
        }
        printf("\n");
    }
    this->client_busy = false;

    // close(sockfd);
}

void Client::run()
{
    ros::Timer timer = n.createTimer(ros::Duration(1 / params.frequency), &Client::iteration, this);
    ros::spin();
}

void Client::get_nodes()
{
    utils::nodes_t nodes_;
    for (int i = 0; i < params.n_robots; i++)
    {
        utils::Node node(i, n);
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
            simulator_utils::Waypoint state = nodes[i].get_state();
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
        // server is completed), wait for the response. Write the response to the
        // variable

        auto con = [this]()
        {
            this->send_recv_data();
        };
        new std::thread(con);
    }
    else
    {
        ROS_DEBUG_STREAM("Iteration skipping. Client busy");
    }
}
