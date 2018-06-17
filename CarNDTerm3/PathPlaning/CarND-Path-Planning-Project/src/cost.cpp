#include <math.h>
#include <cmath>
#include <iostream>

#include "cost.h"
#include "constants.h"

using std::cout;
using std::endl;

// prefer the lane with speed close to the preferred_speed
float lane_speed_cost(float lane_speed, float preferred_speed) {
  float speed_diff = std::abs(lane_speed - preferred_speed);
  return 1 - exp(-speed_diff);
}

// cost of not in the preferred lane: either original lane or the middle lane
float lane_cost(LaneNumber cur_lane, LaneNumber preferred_lane) {
  float lane_diff = std::abs(cur_lane - preferred_lane);
  return lane_diff / 8.0;
}

float free_space_cost(const std::vector<double> &best_free_space, bool is_ego_lane) {
  double free_space_start = best_free_space[0];
  double free_space_end = best_free_space[1];

  double diff_front = free_space_start - MIN_SAFE_DISTRANCE;
  double diff_back = free_space_end + MIN_SAFE_DISTRANCE;
  double diff = free_space_start - free_space_end;

  if(diff < 2 *  MIN_SAFE_DISTRANCE ||  
    diff_front < 0 ||
    diff_back > 0) return 1.0;

  // prefer more free space in the front
  float cost = sqrt(1.0 / (1.0 + WEIGHT_FREESPACE_FRONT * abs(diff_front) + WEIGHT_FREESPACE_BACK * abs(diff_back)));

  return cost;
}

// Compute the total cost if the ego_vehicle is driving on this lane
float calculate_cost(const LaneState &lane_state) {
  
  float speed_cost_ = lane_speed_cost(lane_state.average_speed, MAX_SPEED_MS);
  float lane_cost_ = lane_cost(lane_state.lane, lane_state.ego_lane); // always prefer current lane

  float free_space_cost_ = free_space_cost(lane_state.best_free_space, 
                                           lane_state.lane == lane_state.ego_lane);
  
  float total_cost = WEIGHT_SPEED * speed_cost_ + 
                     WEIGHT_LANE * lane_cost_ +
                     WEIGHT_FREESPACE * free_space_cost_;

  // cout  << "lane: " << lane_state.lane 
  //       << " is_ego_lane: " << (lane_state.lane == lane_state.ego_lane) 
  //       << " speed_cost: " << speed_cost_ 
  //       << " lane_cost: " << lane_cost_
  //       << " free_space_cost: " << free_space_cost_
  //       << " total_cost: " << total_cost
  //       << endl;
  return total_cost;
}