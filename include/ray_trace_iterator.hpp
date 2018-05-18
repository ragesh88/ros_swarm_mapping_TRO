//
// Created by Ragesh on 5/17/18.
//

#ifndef PROJECT_RAY_TRACE_ITERATOR_HPP
#define PROJECT_RAY_TRACE_ITERATOR_HPP

/**
 * This is the header file which defines the prototype of class and functions for performing
 * ray tracing operation on laser beam rays. The main class is a iterator which is derived from
 * iterator_facade class which a part of the iterator module in the boost library.
 */


// Boost libraries
#include <boost/iterator/iterator_facade.hpp>

// C libraries
#include <cassert>
#include <cmath>
#include <cstdlib>
#include <cstdio>


// C++ libraries
#include <limits>
#include <stdexcept>
#include <utility>

namespace NS_occupancy_grid {

template<typename T>
inline int signum(T val) {
  /// A template based implementation signum function.
  /// T is the template type
  /**
   * \param val : the input to the signum function
   *
   */
  return ((T(0) < val) - (val < T(0)));
}

template<typename real_t, typename int_t>
class ray_trace_iterator : public boost::iterator_facade<ray_trace_iterator<real_t, int_t>, std::pair<int_t, int_t>,
                           boost::forward_traversal_tag, std::pair<int_t, int_t>>
{

private:

typedef typename boost::iterator_facade<ray_trace_iterator<real_t, int_t>, std::pair<int_t, int_t>,
boost::forward_traversal_tag, std::pair<int_t, int_t>> super_t;
// Input arguments
real_t px_, py_, dx_, dy_, origin_x, origin_y, cell_size_x, cell_size_y;

// Intermediate variables for faster computation

/// integral steps (direction) (-1, 0, 1) in both x and y
int_t dir_x, dir_y;
/// distance to the nearest grid line
real_t ex_, ey_;
/// Maximum time to collision (from one grid line to next)
real_t Tx_, Ty_;

// State of iterator
/// Grid index
int_t i_, j_;
/// time to collision to next grid line
real_t tx_, ty_;

public:

// constructor
ray_trace_iterator(real_t px, real_t py, real_t dx, real_t dy, real_t origin_x_, real_t origin_y_,
    real_t cell_size_x_, real_t cell_size_y_) :
px_{px}, py_{py},
dx_{dx}, dy_{dy},
origin_x{origin_x_}, origin_y{origin_y_},
cell_size_x{cell_size_x_}, cell_size_y{cell_size_y_} {
/**
 * The constructor for the ray trace iterator.
 *
 * \param px : the start point x coordinate
 * \param py : the start point y coordinate
 * \param dx : the increment along x axis
 * \param dy : the increment along y axis
 * \param origin_x_ : the x coordinate of the map bottom left corner
 * \param origin_y_ : the y coordinate of the map bottom left corner
 * \param cell_size_x_ : the size of the cell along x
 * \param cell_size_y_ : the size of the cell along y
 *
 */

// shift the coordinates to zero the origin
px = px - origin_x_;
py = py - origin_y_;

// grid cell containing (px,py)
i_ = static_cast<int_t>(std::floor(px / cell_size_x_));
j_ = static_cast<int_t>(std::floor(py / cell_size_y_));

dir_x = signum(dx);
dir_y = signum(dy);

// whether the grid line we are going to hit is floor() or ceil()
// depends on the direction in which the ray is moving
// using the fact that ceil() = floor() + 1
int_t floor_or_ceil_x = (dir_x > 0) ? 1 : 0;
int_t floor_or_ceil_y = (dir_y > 0) ? 1 : 0;

// uncomment lines below for debugging
//printf("\n Cell: (%i, %i), dx dy: (%f,%f) \n", i_, j_, dx, dy);

// distance to the nearest grid line
ex_ = std::fabs((i_ + floor_or_ceil_x) * cell_size_x_ - px);
ey_ = std::fabs((j_ + floor_or_ceil_y) * cell_size_y_ - py);

// (max) time to collision from one grid line to another
Tx_ = (dx == 0) ? std::numeric_limits<real_t>::infinity() : cell_size_x_ / std::fabs(dx);
Ty_ = (dy == 0) ? std::numeric_limits<real_t>::infinity() : cell_size_y_ / std::fabs(dy);

// time to collision from this position
tx_ = (dx == 0) ? std::numeric_limits<real_t>::infinity() : ex_ / fabs(dx);
ty_ = (dy == 0) ? std::numeric_limits<real_t>::infinity() : ey_ / fabs(dy);

if (!((tx_ >= 0) && (ty_ >= 0))) {
printf("t:(%f, %f), direction:(%f, %f), position:(%f, %f), cell:(%d, %d), cell size:(%f, %f)\n",
tx_, ty_, dx, dy, px, py, i_, j_, cell_size_x, cell_size_y);
throw std::logic_error("tx < 0 or ty < 0");
}

// time is always positive
assert(tx_ >= 0);
assert(ty_ >= 0);

}

typename super_t::reference dereference() const {
  return std::make_pair(i_, j_);
}

bool equal(ray_trace_iterator it) const {
  /// function to check if the objects are equal
  return ((it.i_ == i_) && (it.j_ == j_) &&
      (it.tx_ == tx_) && (it.ty_ == ty_) &&
      (it.Tx_ == Tx_) && (it.Ty_ == Ty_) &&
      (it.dir_x == dir_x) && (it.dir_y == dir_y));
}

/// overload the == operator
//      friend bool operator== (const ray_trace_iterator<real_t, int_t>& it1,
//                              const ray_trace_iterator<real_t, int_t>& it2);
friend bool operator==(const ray_trace_iterator<real_t, int_t> &it1, const ray_trace_iterator<real_t, int_t> &it2) {
  return ((it1.i_ == it2.i_) && (it1.j_ == it2.j_) &&
      (it1.tx_ == it2.tx_) && (it1.ty_ == it2.ty_) &&
      (it1.Tx_ == it2.Tx_) && (it1.Ty_ == it2.Ty_) &&
      (it1.dir_x == it2.dir_x) && (it1.dir_y == it2.dir_y));
}

void increment();

std::pair<real_t, real_t> real_position() const;

};

template<typename real_t, typename int_t>
void ray_trace_iterator<real_t, int_t>::increment() {
  if (tx_ < ty_) {
    i_ += dir_x;
    ty_ = ty_ - tx_;
    tx_ = Tx_;
  } else {
    j_ += dir_y;
    tx_ = tx_ - ty_;
    ty_ = Ty_;
  }
}

template<typename real_t, typename int_t>
std::pair<real_t, real_t> ray_trace_iterator<real_t, int_t>::real_position() const {

  // whether the grid line we are going to hit is floor() or ceil()
  // depends on the direction ray is moving
  int_t floor_or_ceil_x = (dir_x > 0) ? 1 : 0;
  int_t floor_or_ceil_y = (dir_y > 0) ? 1 : 0;

  real_t ex = (dx_ == 0) ? ex_ // error is same as starting point
                         : tx_ * fabs(dx_);
  real_t ey = (dy_ == 0) ? ey_
                         : ty_ * fabs(dy_);

  real_t px = (i_ + floor_or_ceil_x) * cell_size_x - ex * dir_x;
  real_t py = (j_ + floor_or_ceil_y) * cell_size_y - ey * dir_y;

  // shift coordinates
  px = px + origin_x;
  py = py + origin_y;

  return std::make_pair(px, py);
}

}


#endif //PROJECT_RAY_TRACE_ITERATOR_HPP
