#include "utils.h"
#include <vector>
#include "states_generated.h"

#define MAXLINE 1024

namespace utils = clientutils;

typedef std::vector<flatbuffers::Offset<Agent>> agents_t;

class Client
{
private:
    utils::params_t params;
    ros::NodeHandle n;
    utils::nodes_t nodes;
    flatbuffers::FlatBufferBuilder builder;
    int sockfd;
    bool client_busy;

    void iteration(const ros::TimerEvent &e);
    void send_recv_data(uint8_t *data, uint32_t data_size);
    void get_nodes();
    agents_t get_agent_states();

public:
    Client(utils::params_t, ros::NodeHandle);
    void run();
};
