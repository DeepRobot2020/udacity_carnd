#include "behavior_planner.h"
#include <iostream>
#include "constants.h"
#include "cost.h"
#include "ego_vehicle.h"
#include "utils.h"
#include "spline.h"

using namespace std;

using std::cout;
using std::endl;

BehaviorPlanner::BehaviorPlanner() {
    ev = nullptr;
    fsm_current = fsm_next;
    map_waypoints_x = vector<double> {};
    map_waypoints_y = vector<double> {};
    map_waypoints_s = vector<double> {};
    map_waypoints_dx = vector<double> {};
    map_waypoints_dy = vector<double> {};

    // Waypoint map to read from
    const string map_file_ = MAP_FILE_PATH;

    // Parse the map file and fill up the map waypoints
    ParseMapFile(map_file_, 
                 map_waypoints_x, 
                 map_waypoints_y,
                 map_waypoints_s,
                 map_waypoints_dx,
                 map_waypoints_dy);
}


int BehaviorPlanner::DecideBestLane() {

    if(lane_change_lock > 0) {
      return fsm_current.lane;
    } 

    Traffic           traffic       = ev->traffic;
    vector<LaneState> lane_states   = ev->traffic.lane_state;

    float cur_lane_cost = calculate_cost(lane_states[fsm_current.lane]);

    float best_cost = cur_lane_cost;  
    int   best_lane = fsm_current.lane;



    // Find the best lane by using a costs fuction cf(speed, delta_d, free_space_size)
    for (int cur_lane = 0; cur_lane < LaneNumberTotal; cur_lane++) {
        if(cur_lane == fsm_current.lane) continue;
        // We prefer larger avavg_speed and free_space_length, small delta_d;
        float cur_cost = calculate_cost(lane_states[cur_lane]);

        if (cur_cost + 0.01 < best_cost) {
            best_cost = cur_cost;
            best_lane = cur_lane;
        }
    }
    // If the best lane is 2 lanes aways us, we will try to switch the lane right beside us
    if(best_lane - fsm_current.lane == 2) best_lane --;
    if(fsm_current.lane - best_lane == 2) best_lane ++;

    float best_lane_cost = calculate_cost(lane_states[best_lane]);
    // If the cost of the best lane is no better than us, than do not switch lane
    if(best_lane_cost + 0.01 > cur_lane_cost) {
      return fsm_current.lane;
    }

    return best_lane;
}

void BehaviorPlanner::DecideNextFSMState() {

    const int current_lane = fsm_current.lane;
    const double frenet_d = ev->GetEgoFrenetD();
    const int lane = MapFrenetDToLane(frenet_d);
    const double lane_middle = current_lane * (ROAD_LANE_WIDTH / 2.0) + (current_lane + 1) * (ROAD_LANE_WIDTH / 2.0);

    // Find the best lane by using a costs fuction cf(speed, delta_d, free_space_size)
    int best_lane = DecideBestLane();
    bool is_changeing = fsm_current.lane != fsm_next.lane;


    if(fsm_current.lane != fsm_next.lane
      || lane != fsm_next.lane) {
      cout << "egovehicle is changing from lane " << fsm_current.lane << " -> " << fsm_next.lane << endl;
      return;
    }

    EgoVehileState next_state  = KeepLane;  // default state is always keep current lane

    if (best_lane != fsm_current.lane) {
        if (best_lane == fsm_current.lane - 1)
            next_state = LaneChangeLeft;
        else if (best_lane == fsm_current.lane + 1) {
            next_state = LaneChangeRight;
        } else {
            next_state = KeepLane;
        }
    }

    fsm_next.state = next_state;
    fsm_next.lane  = best_lane;

    cout << "DecideBestLane： current lane " << fsm_current.lane << " best_lane  " << best_lane << endl;
}

static bool isSafeToSwitchLane(const LaneState &lane) {
  std::vector<double>  best_free_space = lane.best_free_space;
  if(best_free_space[0] > MIN_SAFE_DISTRANCE && best_free_space[1] < -MIN_SAFE_DISTRANCE)
    return true;
  return false;
}
void BehaviorPlanner::ProcessFSMState() {

    EgoVehicleState evs      = ev->evs;
    LaneNumber current_lane = MapFrenetDToLane(ev->GetEgoFrenetD());
    if (current_lane == LaneInvalid) {
        cerr << "Invalid ego lane: " << current_lane << endl;
        return;
    }

    Traffic traffic = ev->traffic;
    vector<LaneState> lane_states = ev->traffic.lane_state;
    int current_best_lane = DecideBestLane();

    cout << "ProcessFSMState current lane " << fsm_current.lane 
         << " next_lane  " << fsm_next.lane  
         << " best_lane  " << current_best_lane 
         << endl;

    double speed_diff = 0.0;

    switch (fsm_next.state) {
        case LaneChangeRight:
        case LaneChangeLeft: {
            int target_lane = fsm_next.lane;
            LaneState state = lane_states[target_lane];

            if(target_lane != current_best_lane || !isSafeToSwitchLane(state)) {
              target_lane = current_best_lane;
              // falling back to keep lane
              if(current_best_lane == fsm_current.lane) {
                fsm_next.state = KeepLane;
                current_lane = (LaneNumber)target_lane;
              }
            } 
            // if current speed slower than target lane, speed up
            if(evs.ref_velocity < MAX_SPEED && 
               evs.ref_velocity < state.average_speed) {
              speed_diff += MAX_ACC;
            }

            if(evs.ref_velocity > state.average_speed) {
              fsm_next.state = KeepLane;
              current_lane = (LaneNumber)target_lane;
              lane_change_lock = 10;
            }

        } 
        break;

        default: { 
          break; 
        }
    }

    if(fsm_next.state ==  KeepLane) {

      LaneState current_lane_state = lane_states[current_lane];
      double front_vehilce_s = current_lane_state.best_free_space[0];

      // If there is a in the front quite close to use, decrease speed
      if(front_vehilce_s < MIN_SAFE_DISTRANCE) {
        speed_diff -= MAX_ACC;
      // } else if(front_vehilce_s < 2.0 * MIN_SAFE_DISTRANCE) {
      //   speed_diff -= MAX_ACC / 4;
      } else if(evs.ref_velocity < MAX_SPEED) { // no vehile in front us, speed up
          speed_diff += MAX_ACC;
      }

      fsm_current = fsm_next; 
    } 

    fsm_current.speed_diff = speed_diff;  
}



void BehaviorPlanner::GenerateNextTrajectory(std::vector<double> &x_vals,
                                             std::vector<double> &y_vals) {

    EgoVehicleState &evs = ev->evs;

    lane_change_lock--;
    lane_change_lock = (lane_change_lock < 0) ? 0 : lane_change_lock;

    DecideNextFSMState();
    ProcessFSMState();

    vector<double> ptsx;
    vector<double> ptsy;
    
    const int remaining_traj_size = evs.previous_path_x.size();

    double ref_x   = evs.pos_x;
    double ref_y   = evs.pos_y;
    double ref_yaw = deg2rad(evs.yaw);
    double ref_s = remaining_traj_size ? evs.end_path_s : evs.frenet_s;


    // Any remaining traj points
    if (remaining_traj_size < 2) {

        const double prev_car_x = evs.pos_x - cos(evs.yaw);
        const double prev_car_y = evs.pos_y - sin(evs.yaw);

        ptsx.push_back(prev_car_x);
        ptsx.push_back(evs.pos_x);

        ptsy.push_back(prev_car_y);
        ptsy.push_back(evs.pos_y);

    } else {
        // Use the last two points.
        ref_x = evs.previous_path_x[remaining_traj_size - 1];
        ref_y = evs.previous_path_y[remaining_traj_size - 1];

        const double ref_x_prev = evs.previous_path_x[remaining_traj_size - 2];
        const double ref_y_prev = evs.previous_path_y[remaining_traj_size - 2];

        ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

        ptsx.push_back(ref_x_prev);
        ptsx.push_back(ref_x);

        ptsy.push_back(ref_y_prev);
        ptsy.push_back(ref_y);
    }

    // Setting up target points in the future.
    vector<double> next_wp0 = getXY(ref_s + MIN_SAFE_DISTRANCE, 
                                    2 + 4 * fsm_current.lane, map_waypoints_s,
                                    map_waypoints_x, map_waypoints_y);

    vector<double> next_wp1 = getXY(ref_s + MIN_SAFE_DISTRANCE * 2, 
                                    2 + 4 * fsm_current.lane, map_waypoints_s,
                                    map_waypoints_x, map_waypoints_y);

    vector<double> next_wp2 = getXY(ref_s + MIN_SAFE_DISTRANCE * 3, 
                                    2 + 4 * fsm_current.lane, map_waypoints_s,
                                    map_waypoints_x, map_waypoints_y);

    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]);

    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Making coordinates to local car coordinates.
    for (int i = 0; i < ptsx.size(); i++) {

        double shift_x = ptsx[i] - ref_x;
        double shift_y = ptsy[i] - ref_y;

        ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
        ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
    }

    // Create the spline.
    tk::spline spl;
    spl.set_points(ptsx, ptsy);


    for (int i = 0; i < remaining_traj_size; i++) {
        x_vals.push_back(evs.previous_path_x[i]);
        y_vals.push_back(evs.previous_path_y[i]);
    }

    // Calculate distance y position on 30m ahead.
    double target_x    = MIN_SAFE_DISTRANCE;
    double target_y    = spl(target_x);
    double target_dist = sqrt(target_x * target_x + target_y * target_y);

    double x_add_on = 0;
    double ref_velocity = evs.ref_velocity;

    for (int i = 1; i < PREDICTION_INTERVALS - remaining_traj_size; i++) {

        ref_velocity += fsm_current.speed_diff;
        ref_velocity = min(ref_velocity, MAX_SPEED);
        ref_velocity = max(ref_velocity, MAX_ACC);

        const double N = target_dist / (PREDICTION_SUBDURATION * ref_velocity / 2.24);

        double x_point = x_add_on + target_x / N;
        double y_point = spl(x_point);

        x_add_on = x_point;

        const double x_ref = x_point;
        const double y_ref = y_point;

        x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
        y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

        x_point += ref_x;
        y_point += ref_y;
        
        x_vals.push_back(x_point);
        y_vals.push_back(y_point);
    } // for (int i = 1; i < 50 - remaining_traj_size; i++)

    evs.ref_velocity = ref_velocity;   
}
