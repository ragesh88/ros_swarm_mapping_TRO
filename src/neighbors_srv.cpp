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
// ros libraries
#include <ros/ros.h>
#include <ros/console.h>
#include <nav_msgs/Odometry.h>


// C++ library header files
#include <functional>
#include <iostream>
#include <string>
#include <vector>


// C library header files
#include <stdio.h>

// third party libraries
#include "boost/bind.hpp"

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

    public:

    // constructor

    Neighbors(uint no_of_robots_, std::string robot_name_, std::string topic_);

    // Call back functions
    /// Callback method to Subscribe to the pose of  robots
    void robot_poses_callback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id);

};

///////////////////////////////////////////////////////////////////////////
//                definitions of class Neighbors methods and attributes

// constructor

Neighbors::Neighbors(uint no_of_robots_, std::string robot_name_, std::string topic_):
                    no_of_robots{no_of_robots_},
                    robot_name{robot_name_}
{
  namespace ph = std::placeholders; // adds visibility of _1, _2, _3,...

  // Setting up the subscriber to error free gps pose of each robot
  for(int i =0; i<no_of_robots_; i++){

    sub_robot_pose.emplace_back(nh.subscribe<nav_msgs::Odometry>(robot_name_ + std::to_string(i) + topic_, 10,
                                                                 boost::bind(&Neighbors::robot_poses_callback,this,_1, i)));
  }
}



void Neighbors::robot_poses_callback(const nav_msgs::Odometry::ConstPtr &msg, int robot_id)
{

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
  Neighbors neighbors{static_cast<uint>(std::stoi(no_of_robots)), "robot_", "/base_pose_ground_truth"};
}
