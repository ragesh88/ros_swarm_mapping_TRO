//
// Created by Ragesh on 5/17/18.
//

/**
 * This file contains the definitions for the functions declared in
 * the header file "inverse_sensor_model.h"
 */

#include "inverse_sensor_model.h"

// TODO check the functions to see if everything is compatible

double NS_occupancy_grid::reflectance_model(double grid_range, double range, double max_range, double noise_sd)
/**
 * The function computes probability of occupancy of the grid cell corresponding to grid_range when the
 * ray was reflected.
 * @param grid_range
 * @param range
 * @param max_range
 * @param noise_sd
 * @return
 */
{
  const double start_prob = 0.05;
  const double end_prob = 0.5;
  const double max_prob = 0.8;
  const double init_slope = (end_prob - start_prob) / max_range;
  if (grid_range <= range - 2 * noise_sd - 0.02) {
    return (init_slope * grid_range + start_prob);
  } else {
    return (max_prob);
  }
}

double NS_occupancy_grid::non_reflectance_model(double grid_range, double max_range, double noise_sd)
/**
 * The function computes probability of occupancy of the grid cell corresponding to the grid_range when the
 * ray wasn't reflected
 * @param grid_range
 * @param max_range
 * @param noise_sd
 * @return
 */
{
  const double start_prob = 0.05;
  const double end_prob = 0.5;
  const double init_slope = (end_prob - start_prob) / (max_range - 2 * noise_sd);
  if (grid_range <= max_range - 2 * noise_sd) {
    return (init_slope * grid_range + start_prob);
  } else {
    return (end_prob);
  }

}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////

void NS_occupancy_grid::probability_map_given_measurement_pose(const NS_occupancy_grid::LaserSensor &sensor,
                                                            const int &ray_index,
                                                            std::map<double, cv::Vec<int, 2>> &passed_grids_ranges,
                                                            std::list<std::pair<cv::Vec<int, 2>, double>> &probability)
/**
 * The function computes the probability of the map cells whose coordinates are given as values in the map
 * object passed_grids_ranges.
 * @param sensor
 * @param ray_index
 * @param passed_grids_ranges
 * @param probability
 */
{

// find if reflectance occurred with the ray.
// reflectance occurred if the range of the ray less than
// max range - 2*sqrt(range_noise_const)
bool reflectance = false;
const double noise_variance = sensor.range_noise_const;
if (sensor.ranges[ray_index] < (sensor.range.max - 2 * std::sqrt(noise_variance))) {
reflectance = true;
}

if (reflectance) {
for (const auto &it : passed_grids_ranges) {
double prob = reflectance_model(it.first, sensor.ranges[ray_index], sensor.range.max, noise_variance);
probability.emplace_back(std::pair<cv::Vec<int, 2>, double>(it.second, prob));
}

} else {
for (const auto &it : passed_grids_ranges) {
double prob = non_reflectance_model(it.first, sensor.range.max, noise_variance);
probability.emplace_back(std::pair<cv::Vec<int, 2>, double>(it.second, prob));
}

}
}
