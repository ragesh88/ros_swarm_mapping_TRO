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
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>

// C++ library header files
#include <functional>
#include <iostream>
#include <string>
#include <vector>


// C library header files
#include <stdio.h>

// third party libraries

// local header files
#include "planners.h"
#include "command_line_parser.h"



Class Neighbors{
  /**
   * A class to handle the functionality to find the neighbors of a robot in a specified radius
   */

  /// total number of robots in the swarm
  int no_of_robots;
  /// vector to update the pose of every robot
  std::vector<Pose> robot_poses;

    public:

    // constructor

    neighbors(int no_of_robots_);

    // Call back functions
    /// Callback method to Subscribe to the pose of  robots
    void robot_poses_callback(const nav_msgs::Odometry::ConstPtr& msg, int robot_id);

};


int main()
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
  std::string node_name{"Neighbors_node_" + robot_number};
  ros::init(argc, argv, node_name);
  // display the name of the node
  ROS_INFO_STREAM("Initiated node : "<<ros::this_node::getName());

  // create a Neighbor object

}
