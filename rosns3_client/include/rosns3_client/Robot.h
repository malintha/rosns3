//
// Created by malintha on 10/14/20.
//
#ifndef ROSNS3_ROBOT_H
#define ROSNS3_ROBOT_H

//#include <iostream>
#include "ros/ros.h"
#include "rosns3_client/Waypoint.h"
#include "ros/console.h"

//using namespace std;

class Robot {
    ros::Subscriber state_sub;
    ros::NodeHandle nh;
    void state_cb(const rosns3_client::WaypointConstPtr& wp);

public:
    int id;
    rosns3_client::Waypoint state;
    Robot(int id, const ros::NodeHandle &n);
    rosns3_client::Waypoint get_state() const;


};

#endif
