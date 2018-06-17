#ifndef COST_H
#define COST_H
#include <vector>
#include "traffic.h"


constexpr float WEIGHT_LANE = 0.1;
constexpr float WEIGHT_SPEED = 0.3;
constexpr float WEIGHT_FREESPACE = 0.6;
constexpr float WEIGHT_FREESPACE_FRONT = 0.7;
constexpr float WEIGHT_FREESPACE_BACK = 0.3;

float lane_speed_cost(float lane_speed, float preferred_speed);

// cost of not in the preferred lane: either original lane or the middle lane
float lane_cost(float cur_lane, float preferred_lane);

float free_space_cost(const std::vector<double> &lane_best_free_space);

float calculate_cost(const LaneState &lane_state);

#endif