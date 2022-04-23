# ROSNS3: A Network Simulator (NS-3) bridge for Robot Operating System (ROS) 

This is an NS-3 extension for ROS that allows to simulate network aspects of a robot swarm. Briefly, [NS-3](https://www.nsnam.org/) is an event simulator to design and implement network models. ROSNS-3 allows to
- Simulate wireless signal attenuation,
- Perform network packet routing,
- Calculating recieved signal strength (RSS) between nodes,
- Retrieve updated routing tables in real-time.

Consider citing our work [1] if you find this code helpful for your publications.

## System Overview

![Cover Image](https://github.com/malintha/rosns3_client/blob/master/cover.png?raw=true)

- The ROS and NS-3 environments simulate the robots’ movements, and the network events, respectively.
- ROSNS3 establishes the communication between the two environments using the UDP.
- Currently, to calculate the routing paths as the robots move, we use Optimized Link State Routing (OLSR) algorithm.
- RSS and throughput features are planned for future work.

## Building ROSNS3

ROSNS3 relies on both the client and server modules. To avoid having to install bulky NS-3 software, we have provided a dockerized version of the ROSNS3 server. Unless you need to change the server code, there is no need to explicitly build it.
- Download the docker image by running `docker pull malinthaf/rosns3-server:latest`.
- Run `./run_server.sh` to run the server.
- By default, the rosns3_server will run on the UDP port `28500`.

You can To change the propagation loss model parameters for wireless communiation inside the `run_server.sh` script. To stop the server run `stop_server.sh` script. This will kill the ROSNS3 server, stop the container, and remove the it from memory.

### Build ROSNS3 Client

- Simply run `catkin build rosns3_client`.

## Running Examples

Work in progess.

<!-- ## Using ROSNS3 -->
<!-- ## Changing the Server Side Code

Any changes you will do the server side needs to be pushed into the docker container, and build inside to take effect. We have scripted this process, so there is no need to install NS3 or to do anything manually. Run the `build_server.sh` to push any changes you did to the NS-3 side code. -->

## References

        [1] @ARTICLE{9739846,
            author={Fernando, Malintha and Senanayake, Ransalu and Swany, Martin},
            journal={IEEE Robotics and Automation Letters}, 
            title={CoCo Games: Graphical Game-Theoretic Swarm Control for Communication-Aware Coverage}, 
            year={2022},
            volume={7},
            number={3},
            pages={5966-5973},
            doi={10.1109/LRA.2022.3160968}}
