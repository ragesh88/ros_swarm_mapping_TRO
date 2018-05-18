//
// Created by root on 5/17/18.
//

#ifndef PROJECT_OCCUPANCYGRID_HPP
#define PROJECT_OCCUPANCYGRID_HPP


/**
 * This file contains the definitions of the template class occupancyGrid2D
 */



#include "occupancyGrid.h"


namespace NS_occupancy_grid {

/////////////////////////////////////////////////////////////////////////////////////////////////////////

// Definitions for the members of the class occupancyGrid2D

template<typename real_t, typename int_t>
real_t occupancyGrid2D<real_t, int_t>::nearest_neighbor_distance(cv::Vec<real_t, 2> position,
                                                                 real_t max_range,
                                                                 cv::Vec<int_t, 2> &nearest_neighbor) {


  // Converting the position in to cell coordinates
  int_t i = static_cast<int_t>(std::floor((position(0) - min_pt(0)) / cell_size(0)));
  int_t j = static_cast<int_t>(std::floor((position(1) - min_pt(1)) / cell_size(1)));

  int_t max_range_x = static_cast<int_t>(std::floor(max_range / cell_size(0)));

  // Store the neighbor according to manhattan distance
  std::vector<cv::Vec<int_t, 2>> manhattan_neighbors;

  // Finding the neighbors
  for (int_t r = 0; r < max_range_x; r++) {
    for (int_t xt = std::max(i - r, 0); xt <= std::min(i + r, og_.size[0] - 1); xt++) {

      int_t ry = static_cast<int_t>(std::floor(((r - fabs(xt - i)) * cell_size(0)) / cell_size(1)));

      // y has only two possible values
      for (int_t yt = j - ry; yt <= j + ry; yt += ((2 * ry <= 0) ? 1 : 2 * ry)) {
        // uncomment the line below for debugging
        // printf("(%d, %d), r: %d\n", xt, yt, r);
        if (yt >= og_.size[1] || yt < 0)
          continue;

        if (is_occupied(xt, yt)) {
          manhattan_neighbors.push_back(cv::Vec<int_t, 2>(xt, yt));
          // uncomment the line below for debugging
          //printf("\n added an occupied cell to the list of neighbors");
        }

      }

    }

    if (manhattan_neighbors.size() > 0) {
      break;
    }
  }

  // Finding minimum distance to the occupied neighbor cell
  real_t min_distance = std::numeric_limits<double>::infinity();

  // iterate over the neighbor cells
  for (auto it = manhattan_neighbors.begin(); it != manhattan_neighbors.end(); it++) {
    cv::Vec<real_t, 2> cell_mid_pt{*it};
    // compute cell mid point
    cell_mid_pt += cv::Vec<real_t, 2>(0.5, 0.5);
    cell_mid_pt = cell_mid_pt.mul(cell_size);
    cell_mid_pt += min_pt;
    real_t dist = cv::norm(cell_mid_pt - position);
    if (min_distance < dist) {
      min_distance = dist;
      nearest_neighbor = *it;
    }

    // uncomment the line below for debugging
    // printf("\n Neighbors: (%d, %d), %f \n", (*it)(0), (*it)(1), dist);
  }

  return min_distance;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename real_t, typename int_t>
real_t occupancyGrid2D<real_t, int_t>::ray_trace(real_t px,
                                                 real_t py,
                                                 real_t p_theta,
                                                 real_t max_range,
                                                 cv::Vec<real_t, 2> &final_pos,
                                                 bool &reflectance) {

  real_t dx = std::cos(p_theta);
  real_t dy = std::sin(p_theta);

  ray_trace_iterator<real_t, int_t> ray_trace_it(px, py, dx, dy, min_pt(0), min_pt(1), cell_size(0), cell_size(1));

  real_t dir_mag = std::sqrt(dx * dx + dy * dy);
  real_t n = std::floor(max_range * std::fabs(dx) / (dir_mag * cell_size(0))) +
      std::floor(max_range * std::fabs(dy) / (dir_mag * cell_size(1)));

  int max_size_x = og_.size[0];
  int max_size_y = og_.size[1];

  // iterate using the ray trace iterate object
  for (; n > 0; n--, ray_trace_it++) {

    // grid the coordinates (i,j) as std::pair
    int i = ray_trace_it->first;
    int j = ray_trace_it->second;

    // uncomment the line below to debug
    // printf("\n (%d, %d), (%f, %f) \n", i, j, tx, ty);

    // check if the coordinates are in bounds and is occupied
    if (i < 0 || j < 0 || i >= max_size_x || j >= max_size_y || is_occupied(i, j)) {

      std::pair<real_t, real_t> final_pos_pair = ray_trace_it.real_position();
      final_pos(0) = final_pos_pair.first;
      final_pos(1) = final_pos_pair.second;

      real_t disp_x = final_pos(0);
      real_t disp_y = final_pos(1);
      reflectance = true;
      return std::sqrt(disp_x * disp_x + disp_y * disp_y);
    }
  }

  reflectance = false;
  return max_range;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename real_t, typename int_t>
void occupancyGrid2D<real_t, int_t>::ray_trace_all(real_t px, real_t py, real_t p_theta, real_t max_range,
                                                   std::map<real_t, cv::Vec<int_t, 2>> &all_range_pos)

/// The function return the grid coordinates of all the grids that the ray pass through.
{
  real_t dx = std::cos(p_theta);
  real_t dy = std::sin(p_theta);

  ray_trace_iterator<real_t, int_t> ray_trace_it(px, py, dx, dy, min_pt(0), min_pt(1), cell_size(0), cell_size(1));

  // to check if a grid coordinate is stored twice in the all_range_pos map object
  std::set<cv::Vec<int_t, 2>, vec_comp_class<int_t>> grid_coordinate_check;

  real_t dir_mag = std::sqrt(dx * dx + dy * dy);
  real_t n = std::floor(max_range * std::fabs(dx) / (dir_mag * cell_size(0))) +
      std::floor(max_range * std::fabs(dy) / (dir_mag * cell_size(1)));

  int max_size_x = og_.size[0];
  int max_size_y = og_.size[1];

  // iterate using the ray trace iterate object
  for (; n > 0; n--, ray_trace_it++) {

    // grid the coordinates (i,j) as std::pair
    int i = ray_trace_it->first;
    int j = ray_trace_it->second;

    const std::pair<real_t, real_t> &pos_pair = ray_trace_it.real_position();
    auto range = std::sqrt((pos_pair.first - px) * (pos_pair.first - px) +
        (pos_pair.second - py) * (pos_pair.second - py));

    // uncomment the line below to debug
    //printf("\n (%d, %d), (%f, %f) \n", i, j, pos_pair.first, pos_pair.second);


    // check if the coordinates are in bounds and is occupied
    if (i < 0 || j < 0 || i >= max_size_x || j >= max_size_y || range >= max_range) {

      break;

    }

    // inserting the elements in to the map data structure
    cv::Vec<int_t, 2> grid_coord{i, j}; // value to the map object
    if (!grid_coordinate_check.empty()) { // check if this is the first element to inserted
      // inserted the element only if such a grid coordinate already exist in the map object.
      // this is done by insert the grid coordinates to a set container and checking for duplication
      if (grid_coordinate_check.find(grid_coord) == grid_coordinate_check.end()) {
        grid_coordinate_check.insert(--grid_coordinate_check.end(), grid_coord);
        all_range_pos.insert(--all_range_pos.end(), std::pair<real_t, cv::Vec<int_t, 2>>(range, grid_coord));
      }
    } else { // if this is the first element
      grid_coordinate_check.insert(grid_coord);
      all_range_pos.insert(std::pair<real_t, cv::Vec<int_t, 2>>(range, grid_coord));
    }

  }

}

template<typename real_t, typename int_t>
bool occupancyGrid2D<real_t, int_t>::ray_trace_path(real_t px, real_t py, real_t p_theta, real_t max_range,
                                                    std::map<std::vector<int_t>, std::pair<real_t, real_t>,
                                                    vec_path_comp_class<int_t>> &all_range_pos)
/**
 *  The method performs a ray trace operation and send all the grid coordinates and associated occupancy probability,
 *  range pair as a map object.
 *  return the success of path tracing
 */


{

  real_t dx = std::cos(p_theta);
  real_t dy = std::sin(p_theta);

  ray_trace_iterator<real_t, int_t> ray_trace_it(px, py, dx, dy, min_pt(0), min_pt(1), cell_size(0), cell_size(1));


  real_t dir_mag = std::sqrt(dx * dx + dy * dy);
  real_t n = std::floor(max_range * std::fabs(dx) / (dir_mag * cell_size(0))) +
      std::floor(max_range * std::fabs(dy) / (dir_mag * cell_size(1)));

  int max_size_x = og_.size[0];
  int max_size_y = og_.size[1];

  // grid the coordinates (i,j) as std::pair
  int i = ray_trace_it->first;
  int j = ray_trace_it->second;

  // check if the coordinates are in bounds and is occupied
  if (i < 0 || j < 0 || i >= max_size_x || j >= max_size_y) {

    return false;

  }

  // iterate using the ray trace iterate object
  for (; n > 0; n--, ray_trace_it++) {

    // grid the coordinates (i,j) as std::pair
    i = ray_trace_it->first;
    j = ray_trace_it->second;

    const std::pair<real_t, real_t> &pos_pair = ray_trace_it.real_position();
    auto range = std::sqrt((pos_pair.first - px) * (pos_pair.first - px) +
        (pos_pair.second - py) * (pos_pair.second - py));


    // uncomment the line below to debug
    //printf("\n (%d, %d), (%f, %f) \n", i, j, pos_pair.first, pos_pair.second);


    // check if the coordinates are in bounds and is occupied
    if (i < 0 || j < 0 || i >= max_size_x || j >= max_size_y || range >= max_range) {

      break;

    }

    real_t prob;

    if (get(i,j) == OCCUPIED){
      prob = 0.5;
      //prob = 1.0;
    } else{
      prob = static_cast<real_t>(get(i,j))/ static_cast<real_t>(OCCUPIED);
    }



    // inserting the elements in to the map data structure
    std::vector<int_t> grid_coord{i, j}; // value to the map object

    all_range_pos.emplace(grid_coord, std::pair<real_t, real_t>(prob, range));
  }

  return true;

}

template<typename real_t, typename int_t>
void occupancyGrid2D<real_t, int_t>::map_write(const std::string &filename)
/**
 * The function writes the occupancy grid map to an image file
 * @tparam real_t
 * @tparam int_t
 * @param filename : the filename of the map as a C++ string
 */
{


  cv::Mat og_write ;
  og_.convertTo(og_write, CV_8U);
  for (int row =0; row < og_write.rows; ++row){
    uchar* p = og_write.ptr(row);
    for (int col=0; col<og_write.cols; ++col){
      if (*p == OCCUPIED){
        *p = UNKNOWN;
      }
      p++;
    }
  }



  // showing the image for debugging
//  cv::imshow("output", og_write);
//  cv::waitKey(0);
  cv::imwrite(filename.c_str(), og_write);


}
/////////////////////////////////////////////////////////////////////////////////////////////////////////


template<typename real_t, typename int_t>
void occupancyGrid2D<real_t, int_t>::map_txt_write(const std::string &filename, bool raw)
/**
 *
 * @tparam real_t :  template parameter for real number
 * @tparam int_t : template parameter for integer
 * @param filename : filename with path
 * @param raw : bool to specify if the data has to be written as it is or change OCCUPIED to UNKNOWN
 */
{
  // check if the filename have an extension of .txt if not put it
  auto pos = filename.find_last_of('.');
  std::string new_filename = filename;
  std::string ext = ".txt";
  if(pos == std::string::npos){
    // no extension in the file name
    // then add .csv to file name
    new_filename += ext;
  } else {
    // check if the extension is .csv
    if(filename.substr(pos + 1).compare(ext)){
      // if the extension is not .csv make it .csv
      new_filename = filename.substr(0, pos) + ext;
    }
  }



  // write the matrix to a csv file

  std::ofstream f_out(new_filename);

  if(!f_out) {
    std::cout<<"File not opened \n";
    return;
  }


  for (int row =0; row < og_.rows; ++row){
    uchar* p = og_.ptr(row);
    for (int col=0; col<og_.cols; ++col){
      if (*p == OCCUPIED && (!raw)){
        f_out<< std::to_string(static_cast<uint8_t>(UNKNOWN))<<",";
      }else{
        f_out<< std::to_string(static_cast<uint8_t>(*p))<<",";
      }
      p++;
    }
    f_out<<std::endl;
  }

  // closing the file stream
  f_out.close();

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename real_t, typename int_t>
double occupancyGrid2D<real_t, int_t>::compute_map_entropy()

/**
 * compute the entropy of the map
 * @tparam real_t : template type for real type data
 * @tparam int_t : template type for integer type data
 * @return the entropy of the map
 */
{
  // a lambda function to compute the entropy of the map
//  auto lambda = [&](double a, uint8_t b){ double b_; if(b==OCCUPIED){b_=0.5;} else{b_= static_cast<double>(b)/OCCUPIED;}
//                                         if (b > 0){return (a -b_*std::log(b_)-(1-b_)*std::log(1-b_));} return a;};
//  return std::accumulate(og_.begin<uint8_t>(), og_.end<uint8_t>(), 0.0, lambda);

  // compute with out using lambda functions
  double entropy=0;
  double b_;
  for (int row =0; row < og_.rows; ++row){
    uchar* p = og_.ptr(row);
    for (int col=0; col<og_.cols; ++col){
      if (*p == OCCUPIED ){
        entropy -= std::log(0.5);
      }else{
        b_= static_cast<double>(*p)/OCCUPIED;
        if(b_>0){
          entropy -= b_*std::log(b_)-(1-b_)*std::log(1-b_);
        }
      }
      p++;
    }

  }

  return entropy;

}

/////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename real_t, typename int_t>
double occupancyGrid2D<real_t, int_t>::compute_map_coverage()
/**
 * compute the percentage coverage of the map by the robot
 * @tparam real_t : template type for real type data
 * @tparam int_t : template type for integer type data
 * @return : the percentage coverage
 */
{
  // a lambda function to count the number of the cells covered
//  auto lambda = [&](uint a, uint8_t b){ return a + uint(b/OCCUPIED);};
//  uint count = static_cast<double>(std::accumulate(og_.begin<uint8_t>(),og_.end<uint8_t>(), 0, lambda));
//  return (100.0*(1- static_cast<double>(count)/(og_.rows*og_.cols)));

  double count = 0;
  for (int row =0; row < og_.rows; ++row){
    const uchar* p = og_.ptr(row);
    for (int col=0; col<og_.cols; ++col){
      if (*p == OCCUPIED ){
        count++;
      }
      p++;
    }

  }
  return (100.0*(1-(count)/(og_.rows*og_.cols)));
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////
template<typename int_t>
class vec_comp_class {
 public:
  bool operator()(const cv::Vec<int_t, 2> &t1, const cv::Vec<int_t, 2> &t2) {
    // An operator to compare the cv::Vec<int_t, 2> objects
    if (t1[0] == t2[0] && t1[1] == t2[1])
      return false;
    if (t1[0] < t2[0]) {
      return true;
    } else {
      if (t1[0] == t1[0]) {
        return (t1[1] < t2[1]);
      }
      return false;
    }

  }
};

//////////////////////////////////////////////////////////////////////////////////////////////////////////

template<typename int_t>
class vec_path_comp_class {
  /**
   * key comparison class for ray trace path method in the occupancyGrid2D<real_t, int_t> class
   */
 public:
  bool operator()(const std::vector<int_t> &t1, const std::vector<int_t> &t2) {
    // An operator to compare the cv::Vec<int_t> objects
    if (t1[0] == t2[0] && t1[1] == t2[1])
      return false;
    if (t1[0] < t2[0]) {
      return true;
    } else {
      if (t1[0] == t1[0]) {
        return (t1[1] < t2[1]);
      }
      return false;
    }

  }
};


}


#endif //PROJECT_OCCUPANCYGRID_HPP
