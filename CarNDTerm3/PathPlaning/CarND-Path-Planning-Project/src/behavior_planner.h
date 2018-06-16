
#ifndef BEHAVIOR_PLANNER_H_
#define BEHAVIOR_PLANNER_H_

#include "ego_vehicle.h"
#include <vector>

// forward  declare
class EgoVehicle;

class BehaviorPlanner {

public:
    BehaviorPlanner() {}; 
    BehaviorPlanner(EgoVehicle *ev) : ev(ev) {}; 
    ~BehaviorPlanner() {};
    void SetEgoVehicle(EgoVehicle *ev) { this->ev = ev; };
    void GenerateNextTrajectory(std::vector<double> &x_vals, 
                                std::vector<double> &y_vals);

private:
    EgoVehicle *ev;

};


#endif 

