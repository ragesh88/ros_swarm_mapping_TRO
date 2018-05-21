//
// Created by Ragesh on 5/5/18.
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
 * This cpp file is used to generate executable to that can make the robot move according to a path planned which
 * maximizes the Mutual Information between the map uncertainty and forward ranger sensor model
 */


// ros libraries
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>

// C++ library header files
#include <iostream>
#include <string>


// C library header files
#include <stdio.h>

// third party libraries
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>

// local header files
#include "my_robot.h"
#include "planners.h"
#include "command_line_parser.h"
#include "occupancyGrid.hpp"

// Short name for the namespaces
namespace clp=NS_rag_command_line_parser;
namespace rob=NS_my_robot;
namespace map=NS_occupancy_grid;


int main(int argc, char** argv)
{
  // robot speed parameters
  static const double cruisesSpeed = 0.4;
  static const double turnSpeed = 0.2;

  // parse the inputs
  clp::CommandLineParser cml_parser(argc, argv);

  // Parse input arguments for robot id.
  std::string robot_number;
  if (cml_parser["-id"])
  {
    robot_number = cml_parser("-id");
  }else{
    ROS_ERROR("Need to provide robot id");
  }

  std::string no_of_robots;
  if (cml_parser["-n"])
  {
    no_of_robots = cml_parser("-n");
  }else{
    ROS_ERROR("Need to provide number of robots n");
  }

  // Initializing the ROS node
  std::string node_name{"MI_wanderer_node_" + robot_number};
  ros::init(argc, argv, node_name);
  // display the name of the node
  ROS_INFO_STREAM("Initiated node : "<<ros::this_node::getName());
  // The parameters for map object
  const double min_x = -8; // in meters
  const double min_y = -8; // in meters
  const double cell_size_x = 0.02; // in meters
  const double cell_size_y = 0.02; // in meters
  const int n_cell_x = 800; // no of cells along x
  const int n_cell_y = 800; // no of cells along y

  // Create a map object

  map::occupancyGrid2D<double, int> occ_grid{min_x, min_y, cell_size_x, cell_size_y, n_cell_x, n_cell_y};


  // Velocity object
  Velocity velocity;
  velocity.linear.x=cruisesSpeed;
  velocity.linear.y=0;
  velocity.linear.z=0;
  velocity.angular.x=0;
  velocity.angular.y=0;
  velocity.angular.z=turnSpeed;

  // Create a planner object
  NS_my_planner::base_planner planner{50, 0, velocity};

  // Create a robot object
  rob::Robot robot{static_cast<uint>(std::stoi(robot_number)), std::string{"robot_"}, &planner,
                   static_cast<uint>(std::stoi(robot_number))};

  // Assigning the map object to the robot
  robot.occ_grid_map = &occ_grid;



  // ROS loop rate
  ros::Rate loop_rate(10);

//  geometry_msgs::Twist cmd_vel_msg;
//  cmd_vel_msg.linear.x=0.0;
//  cmd_vel_msg.linear.y=0.0;
//  cmd_vel_msg.linear.z=0.0;
//  cmd_vel_msg.angular.x=0.0;
//  cmd_vel_msg.angular.y=0.0;
//  cmd_vel_msg.angular.z=0.0;

  long unsigned int pre_time=0;

  while (ros::ok()){
    // Publish, Spin and Sleep
    //ROS_INFO("\n Ros time now is %f ", ros::Time::now().toSec());
    //robot.move();
    robot.build_map();
    //robot.merge_map();
    // to do some action every 20 seconds
    long unsigned int time_now = static_cast<long unsigned int>(ros::Time::now().toSec());
    if (time_now%30 == 0 &&
        (pre_time != time_now)){
      //ROS_INFO("\n In condition ");
      //robot.write_map_image();
      pre_time = time_now;
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
}