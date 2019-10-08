//
// Created by Ragesh on 5/5/18.
//

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files in separate categories and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

/***
 * This cpp file is used to generate executable to that can make the robot move according to a path planned which
 * maximizes the Mutual Information between the map uncertainty and forward ranger sensor model. In addition the user
 * can specify the alpha parameter for the MILW and trail number for the experiment. The node saves the required data
 * the folder trail_#(trail number)/robot_#(id)
 */


// ros libraries
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <ros/console.h>

// C++ library header files
#include <iostream>
#include <string>
#include <cstdio>
 #include <bits/stdc++.h>


// C library header files
#include <sys/types.h>
#include <sys/stat.h>


// third party libraries
#include <boost/math/constants/constants.hpp>
#include <boost/filesystem.hpp>
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
namespace bst=boost::filesystem;


int main(int argc, char** argv)
{
  // robot speed parameters
  static const double cruisesSpeed = 0.4;
  static const double turnSpeed = 0.2;
  static const bool DEBUG = false;
  std::string trail_no;

  // set the ros verbosity level to DEBUG for debugging
  if (DEBUG)
  {
      if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
          ros::console::notifyLoggerLevelsChanged();
      }
  }

  // path to store data
  std::string dt_pth{"/home/ragesh/mydisc/data/TRO_ACS/alpha/alpha_"};

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

  //  Parse input arguments for the number of robots in the team
  std::string no_of_robots;
  if (cml_parser["-n"])
  {
    no_of_robots = cml_parser("-n");
  }else{
    ROS_ERROR("Need to provide number of robots n");
  }

  //  Parse input arguments for the alpha parameter
  int alpha = 0;
  std::string alpha_s;
  if (cml_parser["-n"]) {
      alpha = static_cast<int>(std::stod(cml_parser("-a"))*100);
      alpha_s = std::to_string(alpha);
  } else {
      ROS_ERROR("Need to alpha value a");
  }

  // Parse input argument for the trail number
  if (cml_parser["-tr"])
  {
      trail_no = cml_parser("-tr");
  }else{
      ROS_ERROR("Need to provide trail number tr");
  }

  // augment the data path with the given information
  dt_pth +=  alpha_s + "/trail_" + trail_no + "/";

  // check if the directory exists using stat lib
//  struct stat info;
//  if (stat(dt_pth.c_str(), &info) != 0)
//  {
//      ROS_ERROR("cannot access %s\n", dt_pth.c_str());
//  } else{
//      if (info.st_mode & S_IFDIR)
//      {
//          // directory exists
//      } else{
//          // the directory does not exist
//          // create one
//          if (mkdir(dt_pth.c_str(), 0777) == -1)
//          {
//              ROS_ERROR("error in creating the folder");
//          } else{
//              ROS_INFO("Folder created");
//          }
//      }
//  }

  //  check if the directory exists using boost filesystem
  bst::path p(dt_pth);
  // check if the path exist
  if (bst::exists(p))
  {
      // check if the path points to a directory
      if (bst::is_directory(p))
      {
          ROS_INFO("%s exists and is a directory", dt_pth.c_str());
      }
  }else{
      // create the directory if it does not exist
      bst::create_directories(p);
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

  Pose startP{0.0, 0.0, 0.0};

  // radial range sensor noise
  double radial_noise = 0.01;

  // forward sensor model parameter
  NS_my_planner::F_S_M_parameters fsm;
  fsm.sigma = radial_noise;

  // Create a planner object
  NS_my_planner::base_planner planner{50, 0, startP, velocity};
  NS_my_planner::MI_levyWalk_planner MI_planner{0, startP, velocity, fsm, NS_my_planner::CSQMI, 5};

  // Create a robot object
  rob::Robot robot{static_cast<uint>(std::stoi(robot_number)), std::string{"robot_"}, &MI_planner,
                   static_cast<uint>(std::stoi(no_of_robots)), radial_noise};

  // Assigning the map object to the robot
  robot.occ_grid_map = &occ_grid;



  // ROS loop rate
  ros::Rate loop_rate(10);



  long unsigned int pre_time=0;

  while (ros::ok()){
    // Publish, Spin and Sleep
    //ROS_INFO("\n Ros time now is %f ", ros::Time::now().toSec());
    robot.move();
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
    // exist the loop if the time is above a predefined one
//    if (time_now > 200)
//    {
//        break;
//    }
    robot.update_neighbors();
    robot.publish_map();
    ros::spinOnce();
    loop_rate.sleep();
  }
  // compute the coverage value of the map
  robot.add_map_coverage();
  // save the data to file
  robot.write_map_coverage(dt_pth);
}