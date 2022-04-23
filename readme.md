# ROSNS3: A Network Simulator (NS-3) bridge for Robot Operating System (ROS) 

This is an NS-3 extension for ROS that allows to simulate network aspects of a robot swarm. Briefly, [NS-3](https://www.nsnam.org/) is an event simulator to design and implement network models. ROSNS-3 allows to
- Simulate wireless signal attenuation,
- Perform network packet routing,
- Calculating recieved signal strength (RSS) between nodes,
- Retrieve updated routing tables in real-time.

Consider citing our work [1] if you find this code helpful for your publications.

## System Overview

![Cover Image](https://github.com/malintha/rosns3_client/blob/master/cover.png?raw=true)

- The ROS environment simulates the robots’ movements, and the NS-3 environment simulates the network events.
- ROSNS3 establishes the communication between the two environments using the UDP.
- Currently, to calculate the routing paths as the ad-hoc network changes, we use Optimized Link State Routing (OLSR) algorithm.
- RSS and throughput features are planned for future work.


        [1] @ARTICLE{9739846,
            author={Fernando, Malintha and Senanayake, Ransalu and Swany, Martin},
            journal={IEEE Robotics and Automation Letters}, 
            title={CoCo Games: Graphical Game-Theoretic Swarm Control for Communication-Aware Coverage}, 
            year={2022},
            volume={7},
            number={3},
            pages={5966-5973},
            doi={10.1109/LRA.2022.3160968}}