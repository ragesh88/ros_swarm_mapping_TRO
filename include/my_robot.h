//
// Created by Ragesh on 4/25/18.
//

/**
 * This is the header file for my_robot.cpp. The file contains
 * the prototype of the classes and functions for handling the
 * robot functions for simulating it using Stageros
 */

#ifndef MAP_SHARING_INFO_BASED_EXPLORATION_MY_ROBOT_H
#define MAP_SHARING_INFO_BASED_EXPLORATION_MY_ROBOT_H

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

// stage header file added for some functions
//#include <Stage-4.3/stage.hh>

// ros libraries
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>

// C++ library header files
#include <iostream>
#include <string>
#include <vector>


// C library header files
#include <cmath>

// local header files
#include "planners.h"

// typedef types
typedef double meters;



namespace NS_my_robot{

class Robot{


  // Constant Private attributes
  /// Path to save the robot map
  std::string img_path{"./robot"};
  /// The extension indicating the map image format
  const std::string img_type{".png"};
  /// the robot should not exchange information if the
  /// time between their consecutive encounter less than
  /// this value(in seconds)
  const double comm_delay = 10;

  // Private attributes
  /// counter for counting the number of images
  unsigned long image_count = 0;
  /// robot id
  uint robot_id;
  /// name of the robot
  std::string robot_name;
  /// To store the robot velocity as a twist
  geometry_msgs::Twist velocity;
  /// To store and update robot position using true position data
  Pose abs_pose;
  /// Obstacle avoidance variable
  long int avoidCount, randCount;
  /// To display output
  bool verbose = false;
  /// the attribute to store the laser scan message
  std::vector<double> laser_scan;
  /// the robot store their time of encounter with others(seconds)
  std::vector<double> last_communication;
  /// The pointer to the planner
  NS_my_planner::base_planner *planner;


  // Ros variables
  ros::NodeHandle nh;

  // Subscribers
  ros::Subscriber sub_laser_scan;
  ros::Subscriber sub_abs_pose;

  // Publishers
  ros::Publisher pub_cmd_vel;

 public:

  // Constructor
  Robot(uint robot_id_, std::string robot_name_, NS_my_planner::base_planner* planner_);

  /// Static variable to generate robot id
  //static int gen_id;
  ///Static variable to store the pointers to all robots
  /// in the swarm
  static std::vector<Robot *> swarm;

  // Static member function
  /// Static function to update the static variable swarm
  static void swarm_update(Robot* member);
  /// Static function if a robot is close to another robot by a distance d
  static bool any_neighbor(int robot_id, std::vector<int>& neighbors);

  // Laser scanner Callback Method
  void laser_scanner_callback(const sensor_msgs::LaserScan::ConstPtr& msg);

  // base pose ground truth Callback method
  void base_pose_ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg);

  // Publishers
  void publish(geometry_msgs::Twist velocity);
  void publish();

  // velocity update functions
  void set_x_speed(double x);

  void set_y_speed(double y);

  void set_turn_speed(double a);

  void reset_velocity();

  void move();


};
}


#endif //MAP_SHARING_INFO_BASED_EXPLORATION_MY_ROBOT_H
