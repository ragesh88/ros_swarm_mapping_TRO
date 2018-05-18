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

// constructors

Robot::Robot(uint robot_id_, std::string robot_name_, NS_my_planner::base_planner* planner_, double radial_noise):
    robot_id{robot_id_},
    planner{planner_},
    abs_pose{0.0, 0.0, 0.0},
    avoidCount{0},
    randCount{0}

/**
 * Constructor for the robot class
 * @param robot_id_ : the id number of the robot
 * @param laser_sensor_: the parameters for the laser sensor
 * @param robot_name_ : the common name for the robot
 */
{
  robot_name = robot_name_ + std::to_string(robot_id_);

  laser_sensor.range_noise_const = radial_noise;

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////

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

///////////////////////////////////////////////////////////////////////////////////////////////////////////

// callback methods

void Robot::laser_scanner_callback(const sensor_msgs::LaserScan::ConstPtr& msg)
/**
 * This is the callback function to update the laser range measurement data
 * @param msg : the laser range msg as the data type sensor_msgs/LaserScan
 */
{
  if (laser_sensor.ranges.size()==0){ // initialize the object
    laser_sensor.angle.max = msg->angle_max;
    laser_sensor.angle.min = msg->angle_min;
    laser_sensor.range.max = msg->range_max;
    laser_sensor.range.min = msg->range_min;
    // assigning the bearings to the object
    for (double b = msg->angle_min; b <= msg->angle_max; b+= msg->angle_increment){
      laser_sensor.bearings.emplace_back(b);
    }
    int no_of_beams = (msg->angle_max-msg->angle_min)/msg->angle_increment+1;
    for(auto i=0; i<no_of_beams; i++){
      laser_sensor.ranges.emplace_back(static_cast<double>(msg->ranges[i]));
    }
  } else{
    for (auto i=0; i<laser_sensor.ranges.size(); i++){
      laser_sensor.ranges[i] = msg->ranges[i];
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

///////////////////////////////////////////////////////////////////////////////////////////////////////////

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
  auto& laser_scan = laser_sensor.ranges;
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
//    if (verbose && 0)
//      printf("%.3f ", laser_scan[i]);

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
        if ((std::fabs(planner->get_path()->front().des_pose.a - abs_pose.a) < rad_tol &&
            planner->get_path()->front().modes==NS_my_planner::MOTION_MODES::ROTATION_Z) ||
            (std::fabs(planner->get_path()->front().des_pose.x - abs_pose.x) +
            std::fabs(planner->get_path()->front().des_pose.y - abs_pose.y) < 2*rad_tol &&
            planner->get_path()->front().modes==NS_my_planner::MOTION_MODES::TRANSLATION_X)) {
          planner->get_path()->pop();
        }
      } else {
        // Move until the desired motion time is reached up to a tolerance
        if (verbose) {
          std::cout << "\n Motion end time is : " << planner->get_path()->front().motion_end_time << std::endl;
          std::cout << "\n Sim time is : " << ros::Time::now().toSec() << std::endl;
        }
        if (std::fabs(planner->get_path()->front().motion_end_time < ros::Time::now().toSec())) {
          planner->get_path()->pop();
        }
      }
    } else {
      if (verbose)
        std::cout << "\n path generated \n";
      planner->set_startPose(abs_pose);
      try {
        // try to generate a new path
        // TODO change the one below after a planner based on the map is implemented
        planner->generate_path(ros::Time::now().toSec());
      }
      catch (const char *error) {
        std::cerr << error << std::endl;
      }

    }

  }

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void Robot::build_map()
 /**
   * The robot builds an occupancy grid map of the domain using the measurements
   * from the laser range sensor data.
   */
{
  const bool verbose_local = false; // Turn on actions for debugging
  const auto& laserSensor = laser_sensor; // reference to the laser sensor object
  const auto& base_pose = abs_pose; // reference absolute position object
  const int no_of_rays = 10; // number of rays used for updating the occupancy of grid cells
  const int ray_incre = laserSensor.ranges.size()/no_of_rays; // interval in choosing the laser rays

  // Iterate through each ray in the interval ray_incre
  for (int i = 0; i < laserSensor.ranges.size(); i += ray_incre){

    // Get the grid cell coordinate for which the ray passed through
    std::map<double, cv::Vec<int, 2>> passed_grids_ranges;
    occ_grid_map->ray_trace_all(base_pose.x, base_pose.y, laserSensor.bearings[i] + base_pose.a, laserSensor.ranges[i],
                                passed_grids_ranges);

    // compute the probability of occupancy for each grid cell using inverse sensor model for the ray
    std::list<std::pair<cv::Vec<int, 2>, double>> occ_probability;
    NS_occupancy_grid::probability_map_given_measurement_pose(laserSensor, i, passed_grids_ranges, occ_probability);

    for (const auto& it : occ_probability) {
      // In the case of combining probability values for occupancyGrid2D objects
      double v = static_cast<double>(occ_grid_map->get(it.first[0], it.first[1])) /
          static_cast<double>(NS_occupancy_grid::occupancyGrid2D<double, int>::OCCUPIED);

      occ_grid_map->set(it.first[0],
                        it.first[1],
                        static_cast<uint8_t>(NS_occupancy_grid::occupancyGrid2D<double, int>::OCCUPIED *
                            ((it.second) * v)));
    }

  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////


void Robot::write_map_image()
/**
 * The method writes the map object as an image file
 */
{
  std::string count = std::to_string(image_count);
  count = std::string(9 - count.length(), '0') + count;
  std::string filename = data_path + robot_name + "_" + count + img_type;
  //std::cout<<"\n writing map as "<<filename<<std::endl;
  try {
    // Writing as an image is at least 20 times faster than writing to an text file
    //auto start = clock();
    occ_grid_map->map_write(filename);
    //auto stop = clock();
    //std::cout<<"\n the time for image writing is : "<<(stop-start)/double(CLOCKS_PER_SEC)*1000 <<std::endl;

    //occ_grid_map->map_txt_write(filename,myRobot::robot::gen_id);

    image_count++;
  } catch (std::runtime_error &ex) {
    fprintf(stderr, "Exception converting image to PNG format: %s\n", ex.what());

  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////


void Robot::write_map_txt()
/**
 * The method writes the map object as a text file
 */
{
  std::string count = std::to_string(image_count);
  count = std::string(9 - count.length(), '0') + count;
  std::string filename = data_path + robot_name + "_" + count + img_type;
  //std::cout<<"\n writing map as "<<filename<<std::endl;
  try {
    // Writing as an image is at least 20 times faster than writing to an text file
    //auto start = clock();
    occ_grid_map->map_txt_write(filename);
    //auto stop = clock();
    //std::cout<<"\n the time for image writing is : "<<(stop-start)/double(CLOCKS_PER_SEC)*1000 <<std::endl;

    image_count++;
  } catch (std::runtime_error &ex) {
    fprintf(stderr, "Exception writing the image to a text file: %s\n", ex.what());

  }

}


///////////////////////////////////////////////////////////////////////////////////////////////////////////

void Robot::add_map_entropy()
/**
 * the method add the entropy of the map at each instant to a list
 */
{
  double time = ros::Time::now().toSec();
  map_entropy.emplace_back(std::pair<double, double>{time, occ_grid_map->compute_map_entropy()});
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

void Robot::add_map_coverage()
/**
 * method to add the percentage of map covered at each instant to a list
 */
{
  double time = ros::Time::now().toSec();
  map_coverage.emplace_back(std::pair<double, double>{time, occ_grid_map->compute_map_coverage()});
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
