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
 * This cpp file is used to generate executable to that can make the robot move like a wanderer
 */

// ros libraries
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

// C++ library header files
#include <iostream>
#include <string>


// C library header files
#include <stdio.h>

// local header files
#include "my_robot.h"
#include "planners.h"
#include "command_line_parser.h"

// Short name for the namespaces
namespace clp=NS_rag_command_line_parser;
namespace rob=NS_my_robot;


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
    std::string robot_name = "/robot_" + robot_number;
    std::cout << robot_name << std::endl;
  }else{
    ROS_ERROR("Need to provide robot id");
  }

  // Initializing the ROS node
  ros::init(argc, argv,"wanderer_node");

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
  rob::Robot robot{static_cast<uint>(std::stoi(robot_number)), std::string{"robot_"}, &planner};


  // ROS loop rate
  ros::Rate loop_rate(10);

//  geometry_msgs::Twist cmd_vel_msg;
//  cmd_vel_msg.linear.x=0.0;
//  cmd_vel_msg.linear.y=0.0;
//  cmd_vel_msg.linear.z=0.0;
//  cmd_vel_msg.angular.x=0.0;
//  cmd_vel_msg.angular.y=0.0;
//  cmd_vel_msg.angular.z=0.0;

  while (ros::ok()){
    // Publish, Spin and Sleep
    robot.move();
    ros::spinOnce();
    loop_rate.sleep();
  }
}