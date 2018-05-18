//
// Created by Ragesh on 5/17/18.
//

#ifndef PROJECT_INVERSE_SENSOR_MODEL_H
#define PROJECT_INVERSE_SENSOR_MODEL_H

/**
 * This header file defines the classes and function prototype to model the
 * laser range sensor using an inverse sensor model.
 * By inverse sensor model we mean \f$ P(m_i|x,z) \f$.
 * \f$ P(m_i|x,z)\f$ means the probability that a particular grid cell \f$ m_i \f$
 * is occupied given the measurements and position of the robot.
 */

// C++ libraries
#include <iostream>
#include <list>
#include <utility>
#include <vector>

// C libraries
#include <cmath>
#include <cstdio>
#include <cstdlib>

// third party libraries
#include <opencv2/opencv.hpp>


namespace NS_occupancy_grid {

//typedef Stg::ModelRanger::Sensor LaserSensor; // TODO this won't work redefine it

struct Bounds{
  /**
   * A structure to store the bounds of the quantities
   */
  double max;
  double min;
};

// A structure to hold range sensor data
struct LaserSensor{
  /**
   * A structure to store various entities of a laser range scanner
   */
   /// various bearing of the laser scanner
  std::vector<double> bearings;
  /// variance of the range sensor along the radial direction
  double range_noise_const;
  /// the max and min range of the sensor
  Bounds range;
  /// the max and min angle of the sensor
  Bounds angle;
  /// the range along each bearing
  std::vector<double> ranges;

};


void probability_map_given_measurement_pose(const LaserSensor &sensor, const int &ray_index,
                                            std::map<double, cv::Vec<int, 2>> &passed_grids_ranges,
                                            std::list<std::pair<cv::Vec<int, 2>, double>> &probability);

double reflectance_model(double grid_range, double range, double max_range, double noise_sd);

double non_reflectance_model(double grid_range, double max_range, double noise_sd);

}

#endif //PROJECT_INVERSE_SENSOR_MODEL_H
