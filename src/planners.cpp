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

#include"planners.h"

using namespace NS_my_planner;

void base_planner::generate_path(double start_time) {
  via_points point;

  set_planStartTime(start_time);

  // first point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  point.motion_end_time += planTime / 2;
  point.vel_control.linear.x = robotTwist.linear.x;
  point.des_pose = Pose{0.0, 0.0, 0.0};
  point.computed_desPose = false;

  path.push(point);

  // Print the signature of the function in gcc or g++ compiler
  //std::cout<<__PRETTY_FUNCTION__ << ": " <<std::endl ;



}