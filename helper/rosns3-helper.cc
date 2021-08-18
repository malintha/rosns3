#include "rosns3-helper.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>
#include <chrono>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("ROSNS3Server");

neighborhood_t::neighborhood_t(int id, std::vector<int> neighbors): id(id), neighbors(neighbors) {}

ROSNS3Server::ROSNS3Server(int port) {
    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        NS_LOG_ERROR("socket creation failed");
        exit(EXIT_FAILURE);
    }

    memset(&servaddr, 0, sizeof(servaddr));
    memset(&cliaddr, 0, sizeof(cliaddr));

    servaddr.sin_family    = AF_INET; // IPv4
    servaddr.sin_addr.s_addr = INADDR_ANY;
    servaddr.sin_port = htons(port);

    if ( bind(sockfd, (const struct sockaddr *)&servaddr, 
            sizeof(servaddr)) < 0 )
    {
        NS_LOG_ERROR("bind failed");
        exit(EXIT_FAILURE);
    }   
    data = recvdata_t{nullptr, 0};

    NS_LOG_INFO("Initialized server");
}

bool ROSNS3Server::start() {
    this->server_status = true;
    auto server_t = [this]() {
        do {
            socklen_t len;
            len = sizeof(cliaddr); 
            char buffer[BUFFER_LENGTH];
            ssize_t n_bytes = recvfrom(sockfd, buffer, BUFFER_LENGTH, 
                                    MSG_WAITALL, ( struct sockaddr *) &cliaddr, &len);

            const auto p1 = std::chrono::system_clock::now();
            uint timestamp = std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch()).count();
            recvdata_t data_t = {.buffer = buffer, .n_bytes = n_bytes, .timestamp = timestamp};
            this->fifo_list.push_back(data_t);
        }
        while(this->server_running());
    };
    this->server = new std::thread(server_t);
    NS_LOG_INFO("Server started");

    return this->server_status;
}

recvdata_t ROSNS3Server::get_data() {
    recvdata_t return_el = fifo_list.front();
    fifo_list.pop_front();
    return return_el;
}

bool ROSNS3Server::data_ready() {
    return !(this->fifo_list.empty());
}

bool ROSNS3Server::kill() {
    this->server_status = false;
    return this->server_status;
}

bool ROSNS3Server::server_running() {
    return this->server_status;
}

void ROSNS3Server::send_data(uint8_t* data, uint32_t data_size) {
    socklen_t len;
    len = sizeof(cliaddr); 
    sendto(sockfd, data, data_size, MSG_CONFIRM, (const struct sockaddr *) &cliaddr, len);
}

}


