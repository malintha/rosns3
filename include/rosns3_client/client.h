#include "utils.h"
#include <vector>

namespace utils = clientutils;

class Client {
    private:
        utils::params_t params;
        ros::NodeHandle n;
        utils::nodes_t nodes;

        void iteration(const ros::TimerEvent &e);
        void send_data(uint8_t* data, uint32_t data_size);
        void get_nodes();

    public:
        Client(utils::params_t, ros::NodeHandle);
        void run();

};
