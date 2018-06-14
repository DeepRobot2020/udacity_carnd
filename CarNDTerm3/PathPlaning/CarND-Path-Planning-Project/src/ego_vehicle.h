
#ifndef EGOVEHICLE_H
#define EGOVEHICLE_H

#include <vector>
#include "traffic.h"

struct EgoVehicleState {
    double              frenet_s;
    double              frenet_d;
    double              pos_x;
    double              pos_y;
    double              yaw;
    double              speed;
    double              end_path_s;
    double              end_path_d;
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
    };
    ~EgoVehicleState(){};
};

class EgoVehicle {
public:
  EgoVehicle();
  ~EgoVehicle();

  void UpdateState(double frenet_s, double frenet_d, double pos_x, double pos_y,
                   double yaw, double speed, double end_path_s, double end_path_d,
                   const std::vector<double> &previous_path_x, const std::vector<double> &previous_path_y);

  double GetEgoFrenetS() const { return evs.frenet_s; };
  double GetEgoFrenetD() const { return evs.frenet_d; };

  void UpdateTrafficInfo(std::vector<std::vector<double> > sensor_fusion);

private:
    EgoVehicleState evs; 
    Traffic traffic;
};

#endif  // EGOVEHICLE_H