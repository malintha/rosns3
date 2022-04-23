#ifndef ROSNS3_HELPER_H
#define ROSNS3_HELPER_H

#include "ns3/core-module.h"
// #include "ns3/rosns3.h"
#include <sys/socket.h>
#include <sys/types.h>
#include <iostream>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <list>
#include <algorithm>

#define BUFFER_LENGTH 1024

namespace ns3 {

typedef struct recvdata_t {
    char* buffer;
    ssize_t n_bytes;
    uint timestamp;
} recvdata_t;

typedef struct mobile_node_t {
    ns3::Vector position;
    int id;
} mobile_node_t;

typedef struct loss_model_param_t {
    float path_loss_exponent;
    float reference_loss;
    float transmission_power;
    float fading_mean;
    float fading_var;
} loss_model_param_t;

typedef struct neighborhood_t {
    int id;
    std::vector<int> neighbors;

    neighborhood_t(int id, std::vector<int> neighbors);
} neighborhood_t;

class ROSNS3Server {
    public:
        ROSNS3Server(int port);
        char* get_buffer();
        bool start();
        bool kill();
        bool server_running();
        bool data_ready();
        recvdata_t get_data();
        std::list<recvdata_t> fifo_list;
        void send_data(uint8_t* data, uint32_t data_size);

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

