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
#include <functional>
#include <limits>
#include <numeric>
#include <queue>
#include <vector>


// C library header files

// third party libraries
#include <boost/math/constants/constants.hpp>


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

  base_planner(double pTime, double pSTime, Pose P, Velocity V) :
      planTime{pTime},
      planStartTime{pSTime},
      startPose{P},
      robotTwist(V),
      USING_MAP{false} {
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

////////////////////////////////////////////////////////////////////////////////////////////////////////////

typedef double radians;
typedef double meters;

struct F_S_M_parameters{
  /**
   * The structure stores the parameters for the forward sensor
   * for mutual information computation
   */
  /// The maximum distance that a sensor can detect
  meters z_max{2};
  /// The variance of the sensor along the radial direction when modelled as Gaussian
  meters sigma{0.01};
  /// The minimum angle of the laser range sensor wrt to the robot base
  radians min_angle{-M_PI/2};
  /// The maximum angle of the laser range sensor wrt to the robot base
  radians max_angle{M_PI/2};
};

enum REWARD{
  /**
   * enum data type to specify different reward objective to be maximized
   */
  KLDMI,
  CSQMI,
  ENTROPY
};

class MI_levyWalk_planner : public base_planner {
  /**
   * @brief The class implements the path generation for a robot by combining levy walk with
   * the Mutual Information between the sensor model and the map. The basic idea behind the planning
   * is instead of choosing a random direction like in the levy walk setting choose a direction
   * that maximizes the information gain based the mutual information.
   *
   */

  /// to store the forward sensor model parameters of the sensor
  F_S_M_parameters fsm;
  /// the reward function to be maximized
  REWARD reward;
  /// the minimum angle for the Levy walk
  radians min_ang;
  /// the maximum angle for the Levy walk
  radians max_ang;
  /// alpha value for the levy walk
  double alpha;
  /// minimum distance for the levy walk in meters
  meters levy_min;
  /// the distance adjacent via points in a path
  meters dist_btw_path_via;
  /// number of path consider on each side of the robot for computing the mutual information
  int no_path_each_side{3}; // better to keep it odd
  /// number of beams consider for computing the mutual information
  int no_beams_beam_set{8};
  /// store the directions of various beams wrt robot(local coordinates)
  std::vector<radians> beam_dir;

  // private methods
  /// the method to generate the distance to move according to the levy distribution
  meters generate_levy_dist();
  /// the method to generate the intermediate via points as poses for a given direction in the starting pos
  void generate_dir_via_point(const Pose& start_pos, const meters& plan_len, std::queue<Pose>& dir_via_point);
  /// the method to compute the Cauchy Schwarz mutual information of a beam based on
  /// radial Gaussian noise forward range sensor model
  double compute_beam_CSQMI(NS_occupancy_grid::occupancyGrid2D<double, int>* map, double px, double py, double p_theta);
  /// the method to compute the KL divergence mutual information of a beam based on
  /// radial Gaussian noise forward range sensor model
  double compute_beam_KLDMI(NS_occupancy_grid::occupancyGrid2D<double, int>* map, double px, double py, double p_theta);
  /// the method to compute the entropy of the cells traced by a beam
  double compute_beam_Entropy(NS_occupancy_grid::occupancyGrid2D<double, int>* map, double px, double py, double p_theta);

 public:

  // constructor

  MI_levyWalk_planner(double pSTime, Pose P, Velocity V, F_S_M_parameters fsm_, REWARD reward_=KLDMI,
                      meters l_min = 3.0, radians min = -M_PI, radians max = M_PI, double a = 1.5,
                      meters dis_btw_path_via_=2):
      base_planner{0, pSTime, P, V},
      fsm{fsm_},
      reward(reward_),
      min_ang{min},
      max_ang{max},
      alpha{a},
      levy_min{l_min},
      dist_btw_path_via{dis_btw_path_via_}

  /**
   * The constructor with parameters for the class
   * @param pSTime : start time for planning
   * @param P : the start pose for planning
   * @param V : the start velocity for planning
   * @param min : the min angle to rotate
   * @param max : the max angle to rotate
   * @param a : the alpha value of the levy distribution
   * @param l_min : the min distance that robot should move
   * @param dis_btw_path_via_ : the distance adjacent via points in a path
   */
  {


    beam_dir.push_back(fsm.min_angle);
    // generate the directions of the beams to compute MI from [fsm.min_angle fsm.max_angle]
    radians incre_angle = (fsm.max_angle-fsm.min_angle)/(no_beams_beam_set - 1);
    while(beam_dir.back() <= fsm.max_angle){
      beam_dir.push_back(beam_dir.back() + incre_angle);
    }

    /// setting the flag to indicate that the planner uses map
    USING_MAP = true;


  }

  // public methods

  virtual void generate_path(double start_time, NS_occupancy_grid::occupancyGrid2D<double, int>* map);

  virtual ~MI_levyWalk_planner() {}
};

// Defining new namespaces for brevity
namespace b_const = boost::math::constants;

inline double gaussian1D(double x, double mu, double sigma) {
  /// Value of the one dimensional Gaussian function
  return exp(-0.5 * (x - mu) * (x - mu) / (sigma * sigma)) / (sigma * 2*b_const::root_two_pi<double>());
}

}

#endif //MAP_SHARING_INFO_BASED_EXPLORATION_PLANNERS_H
