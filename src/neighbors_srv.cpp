//
// Created by RAGESH on 5/18/18.
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
 * This cpp file is used to generate an executable which is a service node in ROS. The service will
 * accept the id number of a robot along with a sensing radius and returns a vector containing the
 * ids of the robots in that sensing radius. This service is essentially to mimic the communication
 * among robots so that they their maps
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