#include "utils.h"
#include <vector>
#include "states_generated.h"
#include "network_routing_generated.h"

#define MAXLINE 2048

namespace utils = clientutils;

typedef std::vector<flatbuffers::Offset<Agent>> agents_t;
typedef flatbuffers::Vector<flatbuffers::Offset<NetworkNode>> network_t;
typedef std::vector<std::vector<int>> routing_table_t;

typedef struct recv_data {
    ssize_t n_bytes;
    char* recv_buffer;
} recv_data_t; 

class Client
{
private:
    utils::params_t params;
    ros::NodeHandle n;
    utils::nodes_t nodes;
    int sockfd;
    bool client_busy;
    uint8_t *data;
    uint32_t data_size;
    ssize_t n_bytes;
    char recv_buffer[MAXLINE];

    ros::Publisher routing_table_pub;

    routing_table_t set_network();
    routing_table_t routing_tables;
    // const network_t* network;    
    // plot info
    void iteration(const ros::TimerEvent &e);
    recv_data_t* send_recv_data();
    void set_nodes();
    void get_agent_states();
    void log_info();
    void calc_plot_info();
    void publish_routing_table();

public:
    Client(utils::params_t, ros::NodeHandle);
    void run();
    // std::vector<utils::neighborhood_t> get_neighborhoods();
};
