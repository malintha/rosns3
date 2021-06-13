/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */

#include "rosns3-helper.h"
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <arpa/inet.h>

namespace ns3 {

ROSNS3Server::ROSNS3Server(int port) {
    if((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
        perror("socket creation failed");
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
        perror("bind failed");
        exit(EXIT_FAILURE);
    }   
    data = recvdata_t{nullptr, 0};

    std::cout << "Initialized server"<< std::endl;
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
            recvdata_t data_t;
            data_t.buffer = buffer;
            data_t.n_bytes = n_bytes;
            this->data = data_t;
        }
        while(this->get_server_status());

    };
    this->server = new std::thread(server_t);
    std::cout << "Server started"<< std::endl;

    return this->server_status;
}

recvdata_t ROSNS3Server::get_data() {
    return data;
}

bool ROSNS3Server::kill() {
    this->server_status = false;
    return this->server_status;
}

bool ROSNS3Server::get_server_status() {
    return this->server_status;
}

}

