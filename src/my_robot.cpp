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
//namespace planner=NS_my_planner;

Robot::Robot(uint robot_id_, std::string robot_name_, NS_my_planner::base_planner* planner_):
    robot_id{robot_id_},
    planner{planner_},
    abs_pose{0.0, 0.0, 0.0},
    avoidCount{0},
    randCount{0}
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

  // Setting up the subscriber to error free gps pose
  sub_abs_pose = nh.subscribe(robot_name + "/base_pose_ground_truth", 10,
                              &Robot::base_pose_ground_truth_callback, this);

  // Setting up the publisher to command velocity
  pub_cmd_vel = nh.advertise<geometry_msgs::Twist>(robot_name + "/cmd_vel", 100);
}

// publisher methods

void Robot::publish(geometry_msgs::Twist velocity_)
/**
 * Publishes the velocity commands as messages.
 *
 * The name of the topic to which the message is published is defined
 * in the constructor of the robot class.
 * @param velocity_ : the velocity to be published as a Twist
 */
{
  pub_cmd_vel.publish(velocity_);

}

void Robot::publish()
/**
 * Publishes the velocity commands as messages as the attribute velocity
 */
{
  if (verbose){
    std::cout<<"\n publishing the velocity \n";
    std::cout<<"linear x = "<<velocity.linear.x;
    std::cout<<"\n angular z = "<<velocity.angular.z<<std::endl;
  }

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

void Robot::base_pose_ground_truth_callback(const nav_msgs::Odometry::ConstPtr& msg)
/**
 * This is the callback function to update the pose of the robot using error free gps
 * @param msg : the absolute truth of the pose of the robot as nav_msgs/Odometry
 */
{
  abs_pose.x = msg->pose.pose.position.x;
  abs_pose.y = msg->pose.pose.position.y;
  float sin_y = 2.0*(msg->pose.pose.orientation.w * msg->pose.pose.orientation.z +
                     msg->pose.pose.orientation.x * msg->pose.pose.orientation.y);
  float cos_y = 1.0 - 2.0 *(msg->pose.pose.orientation.y * msg->pose.pose.orientation.y +
                            msg->pose.pose.orientation.z * msg->pose.pose.orientation.z);
  abs_pose.a = atan2(sin_y, cos_y);
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
  static NS_my_planner::MOTION_MODES currentMode = NS_my_planner::MOTION_MODES::START;
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
  //if (sample_count < 1)
    //throw "There is no laser data";


  // find the closest distance to the left and right and also check if
  // there's anything in front
  double minleft = 1e6;
  double minright = 1e6;

  for (uint32_t i = 0; i < sample_count; i++) {
    if (verbose && 0)
      printf("%.3f ", laser_scan[i]);

    if ((i > (sample_count / 3)) && (i < (sample_count - (sample_count / 3)))
        && laser_scan[i] < minFrontDistance) {
      if (verbose)
        puts("  obstruction!");
      obstruction = true;
    }

    if (laser_scan[i] < stopDist) {
      if (verbose)
        puts("  stopping!");
      stop = true;
    }

    if (i > sample_count / 2)
      minleft = std::min(minleft, laser_scan[i]);
    else
      minright = std::min(minright, laser_scan[i]);
  }

  if (verbose) {
    puts("");
    printf("min left %.3f \n", minleft);
    printf("min right %.3f\n ", minright);
  }

  if (obstruction || stop || (avoidCount > 0)) {
    // delete the existing planned path
    planner->delete_path();
    currentMode = NS_my_planner::MOTION_MODES::START;
    if (verbose)
      printf("Avoid : %lu\n", avoidCount);

    // setting the robot to stop
    velocity.linear.x = stop ? 0.0 : avoidSpeed;
    publish();

    // once we start avoiding, select a turn direction and stick
    // with it for a few iterations
    if (avoidCount < 1) {
      if (verbose)
        puts("Avoid START");
      avoidCount = random() % avoidDuration + avoidDuration;

      if (minleft < minright) {
        set_turn_speed(-avoidTurn);
        if (verbose)
          printf("turning right %.2f\n", -avoidTurn);
      } else {
        set_turn_speed(+avoidTurn);
        if (verbose)
          printf("turning left %2f\n", +avoidTurn);
      }
      publish();
    }

    avoidCount--;

  } else{

    // if there is no obstruction follow the planned path
    if (verbose)
      puts("\n Cruise \n");

    // check if the a path exist
    if (!planner->get_path()->empty()) {
      if (verbose)
        std::cout << "\n path exist \n";
      // print to debug
      if (verbose) {
        switch (planner->get_path()->front().modes) {
          case NS_my_planner::MOTION_MODES::ROTATION_Z: printf("\nRotation\n");
            //planner->get_path()->front().vel_control.Print("");
            //position->GetVelocity().Print("Actual ");
            //std::cout << "\n The current angle(deg): " << position->GetPose().a * (180 / M_PI) << std::endl;
            std::cout << "\n The desired angle(deg):" << planner->get_path()->front().des_pose.a * (180 / M_PI)
                      << std::endl;
            break;
          case NS_my_planner::MOTION_MODES::TRANSLATION_X: printf("\nTRANSLATION_X\n");
            //planner->get_path()->front().vel_control.Print("");
            break;
          default: printf("\nUndefined mode\n");
        }
      }
      // check if the mode is changed
      if (planner->get_path()->front().modes != currentMode) {
        // instructing the robot in stage to move according to the control
        // publishing to cmd_vel topic
        currentMode = planner->get_path()->front().modes;
        switch (planner->get_path()->front().modes) {
          case NS_my_planner::MOTION_MODES::ROTATION_Z: velocity.linear.x=0;
            velocity.angular.z=planner->get_path()->front().vel_control.angular.z;
            break;
          case NS_my_planner::MOTION_MODES::TRANSLATION_X: velocity.angular.z=0;
            velocity.linear.x = planner->get_path()->front().vel_control.linear.x;
            break;
          default: printf("\nUndefined mode\n");
        }
      }
      publish();
      if (planner->get_path()->front().computed_desPose) {
        // Move until the stage robot has reached the desired orientation up to a tolerance
        if (std::fabs(planner->get_path()->front().des_pose.a - abs_pose.a) < rad_tol) {
          planner->get_path()->pop();
        }
      } else {
        // Move until the desired motion time is reached up to a tolerance
        if (verbose) {
          std::cout << "\n Motion end time is : " << planner->get_path()->front().motion_end_time << std::endl;
          std::cout << "\n Sim time is : " << ros::Time::now().toSec() << std::endl;
        }
        if (std::fabs(planner->get_path()->front().motion_end_time - ros::Time::now().toSec()) < time_tol) {
          planner->get_path()->pop();
        }
      }
    } else {
      if (verbose)
        std::cout << "\n path generated \n";
      planner->set_startPose(abs_pose);
      try {
        // try to generate a new path
        planner->generate_path(ros::Time::now().toSec(), abs_pose);
      }
      catch (const char *error) {
        std::cerr << error << std::endl;
      }

    }

  }

}