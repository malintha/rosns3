#include "client.h"

Client::Client(clientutils::params_t params, ros::NodeHandle n) : params(params), n(n)
{
    get_nodes();

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
    ROS_DEBUG_STREAM("Created "<<params.n_robots << " nodes and subscribers.");

}

void Client::iteration(const ros::TimerEvent &e)
{
    //get data from subscribers
    // subscribers = *get_subscribers();
    
    //create the swarm object

    // send data to the server in a non-blocking call (if the previous call to the
    // server is completed), wait for the response. Write the response to the
    // variable
}
