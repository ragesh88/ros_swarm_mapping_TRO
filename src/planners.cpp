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


/////////////////////////////////////////////////////////////////////////////////////
////////////// Definitions of MI_levyWalk_planner class /////////////////////////////

meters MI_levyWalk_planner::generate_levy_dist()
{
  /**
   * Generate the length from the distribution
   */
  //uniform random number generator
  std::random_device rd;
  std::default_random_engine generator(rd());
  std::uniform_real_distribution<double> distribution(0.0, 1.0);
  // generate a uniform random number between 0 and 1
  auto u = distribution(generator);

  if (alpha == 0) {
    throw "\nLevy alpha should be non zero\n";
  }

  double speed = std::sqrt(get_velocity()->linear.x * get_velocity()->linear.x +
      get_velocity()->linear.y * get_velocity()->linear.y +
      get_velocity()->linear.z * get_velocity()->linear.z);

  if (speed == 0) {
    throw "\nSpeed should be non zero\n";
  }

  return levy_min * pow((1 - u), -1 / alpha); // levy walk distance
}


void MI_levyWalk_planner::generate_dir_via_point(const Pose &start_pos,
                                                 const meters& plan_len,
                                                 std::queue<Pose> &dir_via_point)
/**
 *
 * @param start_pos : The start pose of the path whose via points need to be created
 * @param plan_dis : The length of the path
 * @param dir_via_point : The set of via points as a queue
 */
{
  meters dist_covered{0};
  // check if the queue is empty
  assert(dir_via_point.empty()); // abort if the queue is non empty
  // push the first point into the queue
  dir_via_point.emplace(start_pos);

  while(dist_covered + dist_btw_path_via < plan_len){
    // computing the next via point and pushing it to the queue
    dir_via_point.emplace(Pose{dir_via_point.back().x + dist_btw_path_via*cos(dir_via_point.back().a),
                                    dir_via_point.back().y + dist_btw_path_via*sin(dir_via_point.back().a),
                                    dir_via_point.back().a});
    dist_covered += dist_btw_path_via;
  }
}


double MI_levyWalk_planner::compute_beam_CSQMI(NS_occupancy_grid::occupancyGrid2D<double, int>* map,
                                               double px,
                                               double py,
                                               double p_theta)
/**
 *
 * @param map : a pointer to the map object for ray tracing operations
 * @param px : the x coordinate of the beam base in global frame
 * @param py : the y coordinate of the beam base in global frame
 * @param p_theta : the orientation of the beam in global frame
 * @return : the Cauchy Schwarz Mutual Information for a single beam
 */
{
  // finding the grids and its occupancy probability range traced by the beam
  std::map<std::vector<int>, std::pair<double , double>, NS_occupancy_grid::vec_path_comp_class<int>> traced_grids;
  bool success = map->ray_trace_path(px, py, p_theta, fsm.z_max, traced_grids);

  if(!success){
    return 0.0;
  }

  int delta = 3; // the number of significant cells to compute the double summation

  // compute MI for the ray traced grid cells by the beam

  // pre-compute certain terms to speed up the computation

  // number of beam traced grid cells
  auto no_traced_grid = traced_grids.size();
  // store the probability that the i^{cell} is occupied,
  // by convection prob_0_cell_occ means non of the cells are occupied
  std::vector<double> prob_i_cell_occ(no_traced_grid+1,1);
  std::vector<double> occ_unocc(no_traced_grid+1);
  std::vector<double> w(no_traced_grid+1);

  // computing the probability that i^{th} cell is the first free cell
  for(int i=0; i<prob_i_cell_occ.size(); i++){
    if (i == 0){ // probability that all cells are unoccupied
      for(const auto& it : traced_grids){
        prob_i_cell_occ[i] *= (1-it.second.first);
      }
    } else{
      // compute the probability that i^{th} cell is the first occupied cell
      int j = 1; // cell count
      for(const auto& it : traced_grids){
        if(j<i){ // all i-1 cells are free
          prob_i_cell_occ[i] *= (1-it.second.first);
        }else{
          // the i^{th} cell is occupied
          prob_i_cell_occ[i] *= it.second.first;
          break;
        }
        j++;
      }
    }
  }

  // pre-computing the vector occu_unocc
  int cell_count = 1;
  for(const auto& it : traced_grids){
    occ_unocc[cell_count] =  (it.second.first*it.second.first + (1-it.second.first)*(1-it.second.first));
    cell_count++;
  }

  // pre-computing the vector w
  for (int i=0; i<w.size(); i++){
    w[i] = std::accumulate(occ_unocc.begin() + i +1, occ_unocc.end(),
                           prob_i_cell_occ[i]*prob_i_cell_occ[i], std::multiplies<double>());
  }


  // compute the first term in the MI  computation

  // compute the first term
  double MI_term1 = gaussian1D(0,0,2*fsm.sigma*fsm.sigma);
  MI_term1 *= std::accumulate(w.begin(), w.end(), 0.0);
  if (MI_term1 > 0){
    MI_term1 = std::log(MI_term1);
  }else{
    MI_term1 = 0.0;
  }


  // compute second term
  double MI_term2=std::accumulate(occ_unocc.begin(), occ_unocc.end(), 0.0);

  std::vector<double> discrete_ranges;

  // copy the ranges to the discrete_ranges
  for (const auto& it : traced_grids){
    discrete_ranges.push_back(it.second.second);
  }

  double sum = 0.0;
  for (int j =0; j < prob_i_cell_occ.size(); j++){
    // compute the lower and upper limit for the inner loop
    int lower = j-delta;
    int upper = j+delta;
    if (lower < 0){ // make sure the indices are in bounds
      lower = 0;
    }
    if(upper >= prob_i_cell_occ.size()){
      upper = prob_i_cell_occ.size() -1;
    }
    for (int l = lower;l<=upper; l++){
      if (j==0){
        if (l == 0){
          sum += prob_i_cell_occ[j]*prob_i_cell_occ[l]*gaussian1D(0,0,2*fsm.sigma*fsm.sigma);
        }
        sum += prob_i_cell_occ[j]*prob_i_cell_occ[l]*gaussian1D(discrete_ranges[l-1],
                                                                discrete_ranges.back(),2*fsm.sigma*fsm.sigma);
      } else{
        sum += prob_i_cell_occ[j]*prob_i_cell_occ[l]*gaussian1D(discrete_ranges[l-1],
                                                                discrete_ranges[j-1],2*fsm.sigma*fsm.sigma);
      }

    }
  }

  MI_term2 *= sum;

  if(MI_term2 > 0){
    MI_term2 = std::log(MI_term2);
  } else{
    MI_term2 = 0;
  }


  // compute third term
  double MI_term3=0.0;


  for (int j =0; j < prob_i_cell_occ.size(); j++){
    // compute the lower and upper limit for the inner loop
    int lower = j-delta;
    int upper = j+delta;
    if (lower < 0){ // make sure the indices are in bounds
      lower = 0;
    }
    if(upper >= prob_i_cell_occ.size()){
      upper = prob_i_cell_occ.size() -1;
    }
    for (int l = lower;l<=upper; l++){
      if (j==0){
        if (l == 0){
          MI_term3 += prob_i_cell_occ[j]*w[l]*gaussian1D(0,0,2*fsm.sigma*fsm.sigma);
        }
        MI_term3 += prob_i_cell_occ[j]*w[l]*gaussian1D(discrete_ranges[l-1],
                                                       discrete_ranges.back(),2*fsm.sigma*fsm.sigma);
      } else{
        MI_term3 += prob_i_cell_occ[j]*w[l]*gaussian1D(discrete_ranges[l-1],
                                                       discrete_ranges[j-1],2*fsm.sigma*fsm.sigma);
      }

    }
  }

  if(MI_term3 > 0){
    MI_term3 = -2*std::log(MI_term3);
  } else{
    MI_term3 = 0;
  }

  // return CSQMI
  return (MI_term1+MI_term2+MI_term3);


}



double MI_levyWalk_planner::compute_beam_KLDMI(NS_occupancy_grid::occupancyGrid2D<double, int>* map,
                                               double px,
                                               double py,
                                               double p_theta)
/**
 *
 * @param map : a pointer to the map object for ray tracing operations
 * @param px : the x coordinate of the beam base in global frame
 * @param py : the y coordinate of the beam base in global frame
 * @param p_theta : the orientation of the beam in global frame
 * @return : the KL Diverence Mutual Information for a single beam
 */
{
  // finding the grids and its occupancy probability range traced by the beam
  std::map<std::vector<int>, std::pair<double , double>, NS_occupancy_grid::vec_path_comp_class<int>> traced_grids;
  bool success = map->ray_trace_path(px, py, p_theta, fsm.z_max, traced_grids);

  if(!success){
    return 0.0;
  }

  // compute MI for the ray traced grid cells by the beam

  // pre-compute certain terms to speed up the computation

  // number of beam traced grid cells
  auto no_traced_grid = traced_grids.size();
  // store the probability that the i^{cell} is occupied,
  // by convection prob_0_cell_occ means non of the cells are occupied
  std::vector<double> prob_i_cell_occ(no_traced_grid+1,1);


  // computing the probability that i^{th} cell is the first free cell
  for(int i=0; i<prob_i_cell_occ.size(); i++){
    if (i == 0){ // probability that all cells are unoccupied
      for(const auto& it : traced_grids){
        prob_i_cell_occ[i] *= (1-it.second.first);
      }
    } else{
      // compute the probability that i^{th} cell is the first occupied cell
      int j = 1; // cell count
      for(const auto& it : traced_grids){
        if(j<i){ // all i-1 cells are free
          prob_i_cell_occ[i] *= (1-it.second.first);
        }else{
          // the i^{th} cell is occupied
          prob_i_cell_occ[i] *= it.second.first;
          break;
        }
        j++;
      }
    }
  }

  // the ranges for which the probability P(z_i) is computed (numerical approximation of integral)
  std::vector<double> discrete_ranges;

  // copy the ranges to the discrete_ranges
  for (const auto& it : traced_grids){
    discrete_ranges.push_back(it.second.second);
  }


  // pre-compute probability of a measurement range given the sequence of cell P(z_i| e_k) as a vector of vectors
  // the index set (i,j) represents the value P(z_i|e_j)
  std::vector<std::vector<double>> prob_range_seq(discrete_ranges.size(), prob_i_cell_occ);

  // computing P(z_i| e_k) in index (i,j)
  for (int i =0; i<prob_range_seq.size(); i++){
    for (int j=0; j<prob_range_seq[i].size(); j++){
      if (j==0){
        // case e_0 none of the cells are occupied
        prob_range_seq[i][j] = gaussian1D(discrete_ranges[i], discrete_ranges.back(), fsm.sigma);
      }
      else{
        prob_range_seq[i][j] = gaussian1D(discrete_ranges[i], discrete_ranges[j-1], fsm.sigma);
      }
    }
  }

  // compute and store P(z_i)
  std::vector<double> Prob_range(discrete_ranges.size(),0);
  // also need to find the minimum non zero and index of the zero locations
  double non_zero_min=1e6;
  std::vector<int> zero_index;

  for (int i =0; i<Prob_range.size(); i++){
    for(int j=0; j<prob_i_cell_occ.size(); j++){
      // compute \sigma_j P(z_i|e_j)*P(e_j) which P(z_i)
      Prob_range[i] += prob_range_seq[i][j] * prob_i_cell_occ[j];
    }
    if (std::fabs(Prob_range[i]) <= 2*std::numeric_limits<double>::epsilon() ){ // check if it is zero
      zero_index.push_back(i);
    }else {
      non_zero_min = std::min(non_zero_min, Prob_range[i]);
    }
  }

  // assign the min value to zero probabilities
  for (const auto& i : zero_index){
    Prob_range[i] = non_zero_min;
  }

  // normalize the probability of ranges
  double sum = std::accumulate(Prob_range.begin(), Prob_range.end(), 0.0);


  if (sum > 0){
    for (auto& it : Prob_range){
      it /= sum;
    }
  } else{
    return 0.0;
  }




  // lambda function to compute the MI
  auto lambda = [&](double a, double b){if(b>0){return (a - b*std::log(b));} return a;};

  // compute and return the MI value
  return std::accumulate(Prob_range.begin(), Prob_range.end(), 0.0, lambda);

}


double MI_levyWalk_planner::compute_beam_Entropy(NS_occupancy_grid::occupancyGrid2D<double, int>* map,
                                                 double px,
                                                 double py,
                                                 double p_theta)
/**
 *
 * @param map : a pointer to the map object for ray tracing operations
 * @param px : the x coordinate of the beam base in global frame
 * @param py : the y coordinate of the beam base in global frame
 * @param p_theta : the orientation of the beam in global frame
 * @return : entropy of the cells traced by a single beam
 */
{
  // finding the grids and its occupancy probability range traced by the beam
  std::map<std::vector<int>, std::pair<double , double>, NS_occupancy_grid::vec_path_comp_class<int>> traced_grids;
  bool success = map->ray_trace_path(px, py, p_theta, fsm.z_max, traced_grids);

  if(success){
    double entropy=0;

    for (const auto& it : traced_grids){
      if (it.second.first > 0 && it.second.first < 1){
        entropy += (-it.second.first*std::log(it.second.first) - (1-it.second.first)*std::log(1-it.second.first));
      }
      if (int(it.second.first*10)/10 == 1){
        entropy +=  - 2*(0.5)*std::log(0.5);
      }

    }
    return entropy;
  }

  return 0.0;

}


void MI_levyWalk_planner::generate_path(double start_time, NS_occupancy_grid::occupancyGrid2D<double, int>* map)
/**
 * The virtual method of generating the direction the robot should move to increase information gain
 * @param start_time : the start time when the robot start to plan
 * @param map : the pointer to the map object for ray tracing
 */
{
  //std::cout<<"\nusing map\n";
  bool verbose = false;
  bool debug = false;
  // generate the levy distance that the robot should move
  meters levy_dis = generate_levy_dist();

  // make sure the velocity is non zero
  assert(get_velocity()->linear.x);

  double levy_travel_time = levy_dis/std::fabs(get_velocity()->linear.x);

  const Pose* curPose_t = get_startPose();
  const Pose* curPose = new Pose{curPose_t->x + levy_dis*std::cos(curPose_t->a)/2,
                                           curPose_t->y + levy_dis*std::sin(curPose_t->a)/2,
                                           curPose_t->a};


  std::map<double, std::vector<radians>> dir_MI; // store the compute MI and associated direction

  // path set generation for the robots initialize with current orientation of the robot(global coordinates)
  std::vector<radians> path_directions{curPose->a};
  // adding directions to the left(anti-clockwise) of the robot(global coordinates)
  for(int i = 0; i < (no_path_each_side-1); i++){
    radians new_dir = (i+1)*max_ang/no_path_each_side;
    // adjust the angles to [-M_PI M_PI] when it goes above 180 degree
    if (curPose->a + new_dir > M_PI){
      path_directions.push_back(curPose->a + new_dir -  (2*M_PI));
    } else {
      path_directions.push_back(curPose->a+new_dir);
    }
  }

  // adding directions to the right(clockwise) of robot (global coordinates)
  for(int i=0; i < (no_path_each_side-1); i++){
    radians new_dir = (i+1)*std::fabs(min_ang)/no_path_each_side;
    // adjust the angles the [-M_PI M_PI] when it below -180 degree
    if (curPose->a - new_dir < -M_PI){
      path_directions.push_back(curPose->a + new_dir + 2*M_PI);
    }else{
      path_directions.push_back(curPose->a - new_dir);
    }
  }


  // print for debugging
  if (verbose){
    std::cout<<"\n Path directions"<<std::endl;

    for (const auto& iter : path_directions){
      std::cout<<iter*(180/M_PI)<<std::endl;
    }
  }


  // generate via points for each path dir in path_directions vector
  // and compute mutual information for each path and store in the map dir_MI
  for(const auto& path_dir : path_directions){
    double curr_MI=0.0; // store the mutual information value of the current path
    std::queue<Pose> dir_via_point; // variable to store via point
    Pose startPose{curPose->x, curPose->y, curPose->a + path_dir}; // start pose of the path

    // generate the via points for the path with this start pose
    generate_dir_via_point(startPose, levy_dis, dir_via_point);

    // iterate through various via points in the path
    while(!dir_via_point.empty()){
      Pose& via_point = dir_via_point.front(); // reference to a via point

      // iterate through various beams for that via point
      // compute the mutual information for each beam and add it to the MI of the path
      for(const auto& beam : beam_dir){
        switch (reward){
          case KLDMI: curr_MI += compute_beam_KLDMI(map, via_point.x, via_point.y, via_point.a + beam);
            break;
          case CSQMI: curr_MI += compute_beam_CSQMI(map, via_point.x, via_point.y, via_point.a + beam);
            break;
          case ENTROPY: curr_MI += compute_beam_Entropy(map, via_point.x, via_point.y, via_point.a + beam);
        }
      }

      dir_via_point.pop();

    }

    // the mutual information value and the directions with this value in to the map data structure

    // check if this mutual information value already exist in the map
    auto loc = dir_MI.find(curr_MI); // location of the key curr_MI
    if( loc != dir_MI.end()){
      loc->second.push_back(path_dir); // update vector if the key is already present
    } else { // insert a key value pair in to the map

      dir_MI.emplace(curr_MI, std::vector<radians>{path_dir});

    }

  }

  // find the path directions with maximum mutual information

  std::vector<radians>& max_MI_dirs = dir_MI.rbegin()->second;

  radians des_dir{2*M_PI};

  // pick the direction with minimum turning, if there multiple ones are present
  if(max_MI_dirs.size() > 1){
    for (const auto& dir : max_MI_dirs){
      if (des_dir < std::fabs(dir - curPose->a)){
        des_dir = dir;
      }
    }
  } else {
    des_dir = max_MI_dirs[0];
  }

  // Printing the MI value for debugging
  if (debug){
    std::cout<<"\nPrinting the MI values and directions\n";
    for(const auto& it:dir_MI){
      std::cout<<"["<<it.first<<" : ";
      for (const auto& vec : it.second){
        std::cout<<vec*(180/M_PI)<<", ";
      }
      std::cout<<std::endl;
    }
    std::cout<<"desired direction picked is : "<<des_dir*(180/M_PI)<<std::endl;
  }

  // Pushing the translation motion and rotation motion in to the path

  via_points point;

  // first point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  point.motion_end_time = levy_travel_time/2 + start_time;
  point.vel_control.linear.x = get_velocity()->linear.x;
  point.des_pose = Pose{0.0, 0.0, 0.0};
  point.computed_desPose = false;

  path.push(point); // pushed the translation point

  // second point in the path
  point.modes = MOTION_MODES::ROTATION_Z;
  point.des_pose = Pose{0.0, 0.0, des_dir};
  point.computed_desPose = false;

  const radians &cur_dir = get_startPose()->a; // ref to the current direction
  const auto& w = std::fabs(get_velocity()->angular.z); // omega in magnitude
  double rotate_time = 0;

  // Printing the current direction for debugging
  if (debug){
    std::cout<<"\n The current orientation of the robot is : "<<cur_dir*(180/M_PI)<<std::endl;
  }


  // find the smallest angle to rotate, to find the right \omega
  // case 1 : when both of them are ++ or --
  if (des_dir * cur_dir >= 0) {
    // time to complete the motion
    rotate_time = std::fabs(des_dir - cur_dir) / std::fabs(w);
    if (des_dir > cur_dir) {
      // rotate in anticlockwise direction(left turn)
      Velocity vel;
      vel.linear.x=0;
      vel.linear.y=0;
      vel.linear.z=0;
      vel.angular.x=0;
      vel.angular.y=0;
      vel.angular.z=std::fabs(w);
      point.vel_control = vel;
    } else {
      // rotate in clockwise direction(right turn)
      Velocity vel;
      vel.linear.x=0;
      vel.linear.y=0;
      vel.linear.z=0;
      vel.angular.x=0;
      vel.angular.y=0;
      vel.angular.z=-std::fabs(w);
      point.vel_control = vel;
    }
  }
  // case 2 : when cur_dir is positive and des_dir is negative
  if (cur_dir > 0 && des_dir < 0) {
    // find the smallest angle between the current and the desired directions
    if (std::fabs(cur_dir - des_dir) < std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)) {
      // rotate in clockwise direction(right turn)
      Velocity vel;
      vel.linear.x=0;
      vel.linear.y=0;
      vel.linear.z=0;
      vel.angular.x=0;
      vel.angular.y=0;
      vel.angular.z=-std::fabs(w);
      point.vel_control = vel;
      // time to complete the motion
      rotate_time = std::fabs(cur_dir - des_dir) / std::fabs(w);
    } else {
      // rotate in anticlockwise direction(left turn)
      Velocity vel;
      vel.linear.x=0;
      vel.linear.y=0;
      vel.linear.z=0;
      vel.angular.x=0;
      vel.angular.y=0;
      vel.angular.z=std::fabs(w);
      point.vel_control = vel;
      // time to complete the motion
      rotate_time = (std::fabs(M_PI - cur_dir) + std::fabs(-M_PI - des_dir)) / std::fabs(w);
    }
  }
  // case 3 : when cur_dir is negative and des_dir is positive
  if (cur_dir < 0 && des_dir > 0) {
    // find the smallest angle between the current and desired directions
    if (std::fabs(cur_dir - des_dir) < std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)) {
      // rotate in anticlockwise direction(left turn)
      Velocity vel;
      vel.linear.x=0;
      vel.linear.y=0;
      vel.linear.z=0;
      vel.angular.x=0;
      vel.angular.y=0;
      vel.angular.z=std::fabs(w);
      point.vel_control = vel;
      // time to complete the motion
      rotate_time = std::fabs(cur_dir - des_dir) / std::fabs(w);
    } else {
      // rotate in clockwise direction(right turn)
      Velocity vel;
      vel.linear.x=0;
      vel.linear.y=0;
      vel.linear.z=0;
      vel.angular.x=0;
      vel.angular.y=0;
      vel.angular.z=-std::fabs(w);
      point.vel_control = vel;
      // time to complete the motion
      rotate_time = (std::fabs(-M_PI - cur_dir) + std::fabs(M_PI - des_dir)) / std::fabs(w);
    }
  }

  point.motion_end_time += rotate_time;

  path.push(point); //  pushed the rotation point

  // third point in the path
  point.modes = MOTION_MODES::TRANSLATION_X;
  point.motion_end_time += levy_travel_time;
  point.vel_control.linear.x = get_velocity()->linear.x;
  point.des_pose = Pose{0.0, 0.0, 0.0};
  point.computed_desPose = false;

  path.push(point); // pushed the translation point


}