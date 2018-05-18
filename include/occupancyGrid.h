//
// Created by root on 5/17/18.
//

#ifndef PROJECT_OCCUPANCYGRID_H
#define PROJECT_OCCUPANCYGRID_H


/**
 * This is the header file defines the prototypes for the necessary classes and
 * functions used to store and operate over a 2D occupancy grid map.
 * Every object of the robot class maintains an object of the occupancy grid class
 * in order to perform 2D occupancy grid mapping.
 */



// C libraries
#include <cassert>
#include <cstdio>
#include <cstdlib>

// C++ libraries
#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <stdexcept>
#include <set>

// Third party libraries
#include <boost/math/constants/constants.hpp>
#include <opencv2/opencv.hpp>

// local header files
#include "ray_trace_iterator.hpp" // for ray trace operations


namespace NS_occupancy_grid {

template<typename int_t>
class vec_comp_class;

template<typename int_t>
class vec_path_comp_class;

template<typename real_t, typename int_t>
class occupancyGrid2D {
  /**
   * The occupancyGrid class stores the 2D occupancy map and
   * has the methods to perform various operation on the grid map.
   *
   */
 public:

  /// the minimum coordinate of the map
  cv::Vec<real_t, 2> min_pt{static_cast<real_t>(0), static_cast<real_t>(0)};
  /// the cell size of the grid in meters
  cv::Vec<real_t, 2> cell_size{static_cast<real_t>(0.2), static_cast<real_t>(0.2)};
  /// the matrix to store the occupancy values
  cv::Mat og_;
  /// Value when the map cell is free
  static const uint8_t FREE{0};
  /// Value when the map cell status is unknown
  static const uint8_t UNKNOWN{127};
  /// Value when the map cell is occupied
  static const uint8_t OCCUPIED{255};



  // Constructors


  occupancyGrid2D(real_t min_x, real_t min_y, real_t cell_size_x, real_t cell_size_y, int n_cells_x, int n_cell_y)
      :
      min_pt{min_x, min_y}, cell_size{cell_size_x, cell_size_y}, og_{n_cells_x, n_cell_y, CV_8U, cv::Scalar(OCCUPIED)} {
    /**
     * The constructor for the occupancyGrid2D class.
     *
     * \param min_x : the minimum x coordinate of the actual map
     * \param min_y : the minimum y coordinate of the actual map
     * \param cell_size_x : the size of the cell in the grid map along x in meters
     * \param cell_size_y : the size of the cell in the grid map along y in meters
     * \param n_cell_x : the number of cell in the grid map along x
     * \param n_cell_y : the number of cell in the grid map along y
     *
     */
    {

    }

  }


  // get functions

  uint8_t get(int k) const {
    /// get the value at the \f$k^{the}\f$ location
    int row = k / og_.size[1];
    int col = k % og_.size[1];
    return og_.at<uint8_t>(row, col);
  }

  uint8_t get(int row, int col) const {
    /// get the value at \f$(row,col)\f$ location
    return og_.at<uint8_t>(row, col);
  }



  // set functions

  void set(int k, uint8_t value) {
    /// set the value at the \f $k^{th}\f$ location
    int row = k / og_.size[1];
    int col = k % og_.size[1];
    og_.at<uint8_t>(row, col) = value;

  }

  void set(int row, int col, uint8_t value) {
    /// set the value at \f$(row,col)\f$ location
    og_.at<uint8_t>(row, col) = value;
  }


  // other functions

  bool out_of_bounds(int i, int j) {
    /// the function checks whether the index pair \f$(i,j)\f$ is strictly inside the grid
    return (i >= og_.size[0] || j >= og_.size[1] || i < 0 || j < 0);
  }

  virtual bool is_occupied(int i, int j) {
    /// check if the grid cell in \f$(i,j)\f$ is occupied
    return (og_.at<uint8_t>(i, j) != FREE);
  }

  real_t nearest_neighbor_distance(cv::Vec<real_t, 2> position, real_t max_range,
                                   cv::Vec<int_t, 2> &nearest_neighbor);

  real_t ray_trace(real_t px, real_t py, real_t p_theta, real_t max_range, cv::Vec<real_t, 2> &final_pos,
                   bool &reflectance);

  void ray_trace_all(real_t px, real_t py, real_t p_theta, real_t max_range,
                     std::map<real_t, cv::Vec<int_t, 2>> &all_range_pos);

  bool ray_trace_path(real_t px, real_t py, real_t p_theta, real_t max_range,
                      std::map<std::vector<int_t>, std::pair<real_t, real_t>,
                      vec_path_comp_class<int_t>> &all_range_pos);

  // Writing as an image is at least 20 times faster than writing to an text file

  void map_write(const std::string &filename);

  void map_txt_write(const std::string &filename, bool raw = false);

  double compute_map_entropy();

  double compute_map_coverage();

  cv::Point2i xy2rc(const cv::Vec<real_t, 2> &xy) const {
    /// the function converts the \f$(x,y)\f$ coordinates to corresponding
    /// row and col coordinates of the grid map
    int row = static_cast<int>((xy(0) - min_pt(0)) / cell_size(0));
    int col = static_cast<int>((xy(1) - min_pt(1)) / cell_size(1));
    return cv::Point2i(col, row);
  }

};

}


#endif //PROJECT_OCCUPANCYGRID_H
