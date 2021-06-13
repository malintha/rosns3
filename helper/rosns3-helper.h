/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
#ifndef ROSNS3_HELPER_H
#define ROSNS3_HELPER_H

#include "ns3/rosns3.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <future>

#define BUFFER_LENGTH 1024

namespace ns3 {
struct recvdata_t {
    char* buffer;
    ssize_t n_bytes;
};
class ROSNS3Server {
    public:
        ROSNS3Server(int port);
        char* get_buffer();
        bool start();
        bool kill();
        bool get_server_status();
        recvdata_t get_data();

    private:
        char buffer[BUFFER_LENGTH];
        int sockfd;
        struct sockaddr_in servaddr, cliaddr;
        bool server_status;
        std::thread* server;
        recvdata_t data;


};

}

#endif

