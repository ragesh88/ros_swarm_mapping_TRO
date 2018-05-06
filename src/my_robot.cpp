//
// Created by ragesh on 5/5/18.
//

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// Use using namespace only for the namespace in the .h file corresponding to
// the .cpp file.
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

#include "my_robot.h"

using namespace NS_my_robot;

// Short namespace names
namespace planner=NS_my_planner;

Robot::Robot(uint robot_id_, std::string robot_name_, planner::base_planner* planner_):
    robot_id{robot_id_},
    planner{planner_}
/**
 * Constructor for the robot class
 * @param robot_id_ : the id number of the robot
 * @param robot_name_ : the common name for the robot
 */
{
  robot_name = robot_name_ + std::to_string(robot_id_);

  // initializing the velocity
  velocity.linear.x=0.0;
  velocity.linear.y=0.0;
  velocity.linear.z=0.0;
  velocity.angular.x=0.0;
  velocity.angular.y=0.0;
  velocity.angular.z=0.0;


  // Setting up the subscriber to laser ranger sensors
  sub_laser_scan = nh.subscribe(robot_name + "/base_scan", 10,
                                &Robot::laser_scanner_callback, this);

  // Setting up the publisher to command velocity
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 100);
}

// publisher methods

void Robot::publish(geometry_msgs::Twist velocity)
/**
 * Publishes the velocity commands as messages.
 *
 * The name of the topic to which the message is published is defined
 * in the constructor of the robot class.
 * @param velocity : the velocity to be published as a Twist
 */
{
  pub_cmd_vel.publish(velocity);

}

// callback methods

void Robot::laser_scanner_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
/**
 * This is the callback function to update the laser range measurement data
 * @param msg : the laser range msg as the data type sensor_msgs/LaserScan
 */
{
  if (laser_scan.size()==0){
    int no_of_beams = (msg->angle_max-msg->angle_min)/msg->angle_increment+1;
    for(auto i=0; i<no_of_beams; i++){
      laser_scan.emplace_back(static_cast<float>(msg->ranges[i]));
    }
  } else{
    for (auto i=0; i<laser_scan.size(); i++){
      laser_scan[i] = msg->ranges[i];
    }
  }
}


// velocity update function
void Robot::set_x_speed(double x)
/**
 * Set the speed of the robot in x direction
 * @param x : speed in x direction
 */
{
  velocity.linear.x = x;
}

void Robot::set_y_speed(double y)
/**
 * Set the speed of the robot in y direction
 * @param y : speed in y direction
 */
{
  velocity.linear.y = y;
}

void Robot::set_turn_speed(double a)
/**
 * Set the rotation speed of the robot in z direction
 * @param a : rotation speed of the  robot
 */
{
  velocity.angular.z = a;
}

void Robot::reset_velocity()
/**
 * Reset the velocity to zero
 */
{
  velocity.linear.x = 0;
  velocity.linear.y = 0;
  velocity.linear.z = 0;
  velocity.angular.x = 0;
  velocity.angular.y = 0;
  velocity.angular.z = 0;
}

void Robot::move()
/**
 *
 */
{
  /// The function moves the robot according to the planner object

  // Tolerance to check the various angle and time conditions
  const double rad_tol = 2 * M_PI / 180;
  const double time_tol = 0.1;
  static planner::MOTION_MODES currentMode = MOTION_MODES::START;
  // variables for obstacle avoidance
  static const double stopDist = 0.3; // stopping distance of the robot
  static const int avoidDuration = 10; // duration to perform obstacle avoidance
  static const double avoidSpeed = 0.05;
  static const double avoidTurn = 0.5;
  static const double minFrontDistance = 1.0;
  bool obstruction = false; //flag for detecting obstruction
  bool stop = false; // flag to stop the robot

  // get the laser data

  auto sample_count = laser_scan.size();
  if (verbose)
    printf("\n sample count laser :%lu \n", sample_count);
  if (sample_count < 1)
    throw "There is no laser data";


  // find the closest distance to the left and right and also check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++) {
    if (verbose && 0)
      printf("%.3f ", scan[i]);

    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
        && scan[i] < minFrontDistance) {
      if (verbose)
        puts("  obstruction!");
      obstruction = true;
    }

    if (scan[i] < stopDist) {
      if (verbose)
        puts("  stopping!");
      stop = true;
    }

    if (i > sample_count / 2)
      minleft = std::min(minleft, scan[i]);
    else
      minright = std::min(minright, scan[i]);
  }
  
}