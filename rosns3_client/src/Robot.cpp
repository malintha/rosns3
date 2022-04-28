//
// Created by malintha on 10/14/20.
//
#include <iostream>
#include "Robot.h"

using namespace std;

rosns3_client::Waypoint Robot::get_state() const {
    // ROS_DEBUG_STREAM("Robot: "<<id<<" pos: "<<this->state.position.x<<" " << this->state.position.y);
    return this->state;
}

void Robot::state_cb(const rosns3_client::WaypointConstPtr &wp) {
    this->state.position = wp->position;
    this->state.velocity = wp->velocity;
    this->state.acceleration = wp->acceleration;
}

// TODO: pass the topic prefix as a parameter
Robot::Robot(int id, const ros::NodeHandle &n):id(id), nh(n) {

    stringstream ss;
    ss << "/robot_"<<to_string(id)<<"/current_state";
    this->state_sub = nh.subscribe(ss.str(), 10,
                                   &Robot::state_cb,
                                   this);

    ROS_DEBUG_STREAM("Robot: "<<this->id<<" Waiting for robot states.");
    ros::topic::waitForMessage<rosns3_client::Waypoint>(ss.str(), ros::Duration(3));
}
