
#ifndef EGOVEHICLE_H
#define EGOVEHICLE_H

#include <vector>
#include "traffic.h"
#include "behavior_planner.h"

// class BehaviorPlanner;

struct EgoVehicleState {
    double              frenet_s;
    double              frenet_d;
    double              pos_x;
    double              pos_y;
    double              yaw;
    double              speed;
    double              end_path_s;
    double              end_path_d;
    double              ref_velocity;

    std::vector<double> previous_path_x;
    std::vector<double> previous_path_y;
    
    EgoVehicleState() {
        frenet_s   = 0.0;
        frenet_d   = 0.0;
        pos_x      = 0.0;
        pos_y      = 0.0;
        yaw        = 0.0;
        speed      = 0.0;
        end_path_s = 0.0;
        end_path_d = 0.0;
        ref_velocity = 0.0;
        previous_path_x = {};
        previous_path_y = {};
    };

    EgoVehicleState(const EgoVehicleState &rhs) {
        frenet_s   = rhs.frenet_s;
        frenet_d   = rhs.frenet_d;
        pos_x      = rhs.pos_x;
        pos_y      = rhs.pos_y;
        yaw        = rhs.yaw;
        speed      = rhs.speed;
        end_path_s = rhs.end_path_s;
        end_path_d = rhs.end_path_d;
        ref_velocity = rhs.ref_velocity;
        previous_path_x = std::move(rhs.previous_path_x);
        previous_path_y = std::move(rhs.previous_path_y);
    }

    EgoVehicleState& operator = (const EgoVehicleState &rhs) {
        frenet_s   = rhs.frenet_s;
        frenet_d   = rhs.frenet_d;
        pos_x      = rhs.pos_x;
        pos_y      = rhs.pos_y;
        yaw        = rhs.yaw;
        speed      = rhs.speed;
        end_path_s = rhs.end_path_s;
        end_path_d = rhs.end_path_d;
        ref_velocity = rhs.ref_velocity;
        previous_path_x = std::move(rhs.previous_path_x);
        previous_path_y = std::move(rhs.previous_path_y);
    }

    ~EgoVehicleState(){};
};

class EgoVehicle {
public:
  EgoVehicle();
  ~EgoVehicle();

  double GetEgoFrenetS() const { return evs.frenet_s; };
  double GetEgoFrenetD() const { return evs.frenet_d; };


  void UpdateState(double frenet_s, double frenet_d, double pos_x, double pos_y,
                   double yaw, double speed, double end_path_s, double end_path_d,
                   const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);
  void UpdateTrafficInfo(const std::vector<std::vector<double>> &sensor_fusion);
  void GenerateNextTrajectory(std::vector<double> &x_vals, std::vector<double> &y_vals);
  EgoVehicleState evs; 
  Traffic traffic;
};

#endif  // EGOVEHICLE_H