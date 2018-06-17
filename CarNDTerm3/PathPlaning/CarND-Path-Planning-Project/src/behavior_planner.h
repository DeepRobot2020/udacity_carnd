
#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include <vector>
#include "ego_vehicle.h"

// forward  declare
class EgoVehicle;

struct BehaviorPlannerFSM {
  double speed_diff;
  int lane;
  EgoVehileState state;
  BehaviorPlannerFSM() {
    speed_diff = 0.0;
    state = KeepLane;
    lane = LaneMiddle;
  };
  ~BehaviorPlannerFSM() {};
};

class BehaviorPlanner {

   public:
    BehaviorPlanner();
    BehaviorPlanner(EgoVehicle *ev) : ev(ev){};
    ~BehaviorPlanner(){};
    void SetEgoVehicle(EgoVehicle *ev) { this->ev = ev; };

    void GenerateNextTrajectory(std::vector<double> &x_vals, std::vector<double> &y_vals);
    void DecideNextFSMState();
    void ProcessFSMState();
    int DecideBestLane();
    
   private:
    int lane_change_lock = 10;
    EgoVehicle *   ev;
    BehaviorPlannerFSM fsm_current;
    BehaviorPlannerFSM fsm_next;

    std::vector<double> map_waypoints_x;
    std::vector<double> map_waypoints_y;
    std::vector<double> map_waypoints_s;
    std::vector<double> map_waypoints_dx;
    std::vector<double> map_waypoints_dy;
};

#endif
