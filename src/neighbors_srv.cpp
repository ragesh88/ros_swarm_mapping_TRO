//
// Created by RAGESH on 5/18/18.
//

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////


/***
 * This cpp file is used to generate an executable which is a service node in ROS. The service will
 * accept the id number of a robot along with a sensing radius and returns a vector containing the
 * ids of the robots in that sensing radius. This service is essentially to mimic the communication
 * among robots so that they their maps
 */


// ros libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>

// service header
#include "map_sharing_info_based_exploration/neighbors.h"


// C++ library header files
#include <functional>
#include <iostream>
#include <string>
#include <vector>


// C library header files
#include <stdio.h>

// third party libraries
#include <boost/bind.hpp>

// local header files
#include "planners.h"
#include "command_line_parser.h"

// Short name for the namespaces
namespace clp=NS_rag_command_line_parser;



class Neighbors{
  /**
   * A class to handle the functionality to find the neighbors of a robot in a specified radius
   */

  /// total number of robots in the swarm
  uint no_of_robots;
  /// name of the robot
  std::string robot_name;
  /// vector to update the pose of every robot
  std::vector<Pose> robot_poses;

    // Ros variables
    ros::NodeHandle nh;

    // Subscribers
    std::vector<ros::Subscriber> sub_robot_pose;

    // service
    ros::ServiceServer nbh_service;

    public:

    // constructor

    Neighbors(uint no_of_robots_, std::string robot_name_, std::string topic_, std::string service_name);

    // Call back functions
    /// Callback method to Subscribe to the pose of  robots
    void robot_poses_callback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id);
    /// Callback method to obtain the neighbors of the robot
    bool get_robot_neighbour(map_sharing_info_based_exploration::neighbors::Request& req,
                             map_sharing_info_based_exploration::neighbors::Response& res);


};

///////////////////////////////////////////////////////////////////////////
//                definitions of class Neighbors methods and attributes

// constructor

Neighbors::Neighbors(uint no_of_robots_, std::string robot_name_, std::string topic_, std::string service_name):
                    no_of_robots{no_of_robots_},
                    robot_name{robot_name_}
{

  // initializing the pose vector
  for(int i = 0; i<no_of_robots_; i++){
    robot_poses.emplace_back(Pose{0.0, 0.0, 0.0});
  }

  // Setting up the service
  nbh_service = nh.advertiseService(service_name, &Neighbors::get_robot_neighbour, this);

  // Setting up the subscriber to error free gps pose of each robot
  for(int i =0; i<no_of_robots_; i++){

    sub_robot_pose.emplace_back(nh.subscribe<nav_msgs::Odometry>(robot_name_ + std::to_string(i) + topic_, 10,
                                                                 boost::bind(&Neighbors::robot_poses_callback,this,_1, i)));
  }
}

// callback function definition

void Neighbors::robot_poses_callback(const nav_msgs::Odometry::ConstPtr &msg, int robot_id)
{
  robot_poses[robot_id].x = msg->pose.pose.position.x;
  robot_poses[robot_id].y = msg->pose.pose.position.y;
  // angle is not updated as it is not required

  // uncomment the lines below for debugging
//  ROS_INFO("Position of robot %d updated", robot_id);
//  ROS_INFO("x : %f, y : %f", robot_poses[robot_id].x, robot_poses[robot_id].y);

}

bool Neighbors::get_robot_neighbour(map_sharing_info_based_exploration::neighbors::Request &req,
                                    map_sharing_info_based_exploration::neighbors::Response &res)
/**
 * The method finds the numbers of robots the are with in a particular radius of a robot
 * @param req : request object of the neighbors service
 * @param res : response object of the neighbors service
 * @return : whether the service request was processed or not
 */
{
  double rad2 = req.radius*req.radius; // square of radius
  for(int i=0; i<no_of_robots; i++){
    double dist2 = (robot_poses[i].x-robot_poses[req.robot_id].x)*(robot_poses[i].x-robot_poses[req.robot_id].x) +
                   (robot_poses[i].y-robot_poses[req.robot_id].y)*(robot_poses[i].y-robot_poses[req.robot_id].y);
    if (dist2 < rad2  && i != req.robot_id){
      res.neighbors.emplace_back(i);
    }
  }
}


/////////////////////////////////////////////////////////////////////////////


int main(int argc, char** argv)
{
  // parse the inputs
  clp::CommandLineParser cml_parser(argc, argv);

  // Parse input arguments for robot id.
  std::string no_of_robots;
  if (cml_parser["-n"])
  {
    no_of_robots = cml_parser("-n");
  }else{
    ROS_ERROR("Need to provide number robots using -n option");
  }

  // Initializing the ROS node
  std::string node_name{"Neighbors_node" };
  ros::init(argc, argv, node_name);
  // display the name of the node
  ROS_INFO_STREAM("Initiated node : "<<ros::this_node::getName());

  // create a Neighbors object
  Neighbors neighbors{static_cast<uint>(std::stoi(no_of_robots)), "robot_", "/base_pose_ground_truth", "robot_neighbors"};

  // ROS loop rate
  ros::Rate loop_rate(10);

  ros::spin();

//  while (ros::ok()){
//    // Publish, Spin and Sleep
//
//    ros::spinOnce();
//    loop_rate.sleep();
//  }
}
