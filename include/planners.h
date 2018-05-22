//
// Created by Ragesh on 5/4/18.
//

#ifndef MAP_SHARING_INFO_BASED_EXPLORATION_PLANNERS_H
#define MAP_SHARING_INFO_BASED_EXPLORATION_PLANNERS_H

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////


// ros libraries
#include <ros/ros.h>
#include "geometry_msgs/Vector3.h"
#include <geometry_msgs/Twist.h>

// C++ library header files
#include <queue>
#include <vector>


// C library header files


// local header files
#include "occupancyGrid.hpp"

// typedef types
typedef double meters;
typedef double radians;
typedef geometry_msgs::Twist Velocity;

struct Pose{
  meters x;
  meters y;
  radians a;
};

/**
 * The planner is an abstract class for all planner for the robot.
 * Any planner written for the robot should inherit from base planner
 *
 */

////////////////////////////////////////////////////////////////////////////////
// My naming conventions for the project
// Put header files separated different category and in alphabetical order
// unless there some compatibility issue between the header files
// macros and const : ALL_IN_CAPS
// namespace : NS_prefix_small_letter_with_underscore
// class and structs : First_letter_caps_with_underscore
// functions and variables: all_small_letters
/////////////////////////////////////////////////////////////////////////////////

namespace NS_my_planner {

enum MOTION_MODES {

  /**
   * \brief The enum structure MOTION_MODES enumerates the various modes
   * of motion encoded in a via point of a motion path.
   *
   * \param START : the via point with no velocity
   * \param ROTATION_Z : the via point with only rotational velocity along z direction
   * \param TRANSLATION : the via point with translational velocity
   * \param TRANSLATION_X : the via point with only translational velocity along the x direction wrt to robot
   * \param TRANSLATION_Y : the via point with only translational velocity along the y direction wrt to robot
   * \param ALL : the via point with all the velocities
   */

      START,
  ROTATION_Z,
  TRANSLATION,
  TRANSLATION_X,
  TRANSLATION_Y,
  ALL
};

struct via_points {
  /**
   * \brief The via points structure is used to store each via point in the planned
   *
   * \param modes : the motion mode corresponding to the via point
   * \param motion_end_time : the time till the current control action to be done
   * \param vel_control : the control action for the current via point
   * \param des_pose : the desired pose for the via point
   * \param computed_desPose : flag to check whether the desired pose is computed
   */
  MOTION_MODES modes = START;
  double motion_end_time = 0;
  Velocity vel_control;
  Pose des_pose{0.0, 0.0, 0.0};
  bool computed_desPose = false;
};

/// a type defined type path for storing path
typedef std::queue<via_points> Path;

class base_planner {
  /**
   * \param planTime : the time for which motion has to be planned
   * \param planStartTime : the start time of the planner
   * \param startPose : the pose of the robot at the start of planning
   * \param robotTwist : the velocity of the robot for planning
   * \param path : the planned path
   */
  double planTime = 0;
  double planStartTime = 0;
  Pose startPose {0.0, 0.0, 0.0};
  Velocity robotTwist;

 protected:

  Path path;
  /// check if the planner is using map
  bool USING_MAP;

 public:

  // constructors

  base_planner() {
    /**
     * Default constructor with no arguments
     */

  }

  base_planner(double pTime, double pSTime, Velocity V) :
      planTime{pTime},
      planStartTime{pSTime},
      robotTwist(V) {
    /**
     * constructor with parameters
     * \param pTime : the time for which motion has to be planned
     * \param pSTime : the start time of the planner
     */

  }

  // get functions

  double get_planTime() const {
    return planTime;
  }

  double get_planStartTime() const {
    return planStartTime;
  }

  const Pose *get_startPose() const {
    return &startPose;
  }

  const Velocity *get_velocity() const {
    return &robotTwist;
  }

  Path *get_path() {
    return &path;
  }

  bool is_using_map(){
    /// check if the planner is using a map for planning
    return USING_MAP;
  }

  // set functions

  void set_planTime(double time) {
    planTime = time;
  }

  void set_planStartTime(double time) {
    planStartTime = time;
  }

  void set_startPose(const Pose &P) {
    startPose = P;
  }

  void set_robotTwist(const Velocity &V) {
    robotTwist = V;
  }

  // other functions
  ///The function generates path for a base planner
  virtual void generate_path(double start_time);

  /// The function generates path for a derive planner class
  /// the non trivial implementation can be found in MI_levyWalk_planner class
  virtual void generate_path(double start_time, NS_occupancy_grid::occupancyGrid2D<double, int>* map){}

  void delete_path() {
    /// The function deletes the path stored in the variable
    while (!path.empty()) path.pop();

  }

  virtual ~base_planner() {}

};

}

#endif //MAP_SHARING_INFO_BASED_EXPLORATION_PLANNERS_H
