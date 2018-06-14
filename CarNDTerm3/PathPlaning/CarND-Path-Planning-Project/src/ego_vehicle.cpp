#include "ego_vehicle.h"

using namespace std;


EgoVehicle::EgoVehicle() {};

EgoVehicle::~EgoVehicle() {};

void EgoVehicle::UpdateState(double frenet_s, double frenet_d, double pos_x, double pos_y,
                             double yaw, double speed, double end_path_s, double end_path_d,
                             const std::vector<double> &previous_path_x,
                             const std::vector<double> &previous_path_y) {

    evs.frenet_s        = frenet_s;
    evs.frenet_d        = frenet_d;
    evs.pos_x           = pos_x;
    evs.pos_y           = pos_y;
    evs.yaw             = yaw;
    evs.speed           = speed;
    evs.end_path_s      = end_path_s;
    evs.end_path_d      = end_path_d;
    evs.previous_path_x = previous_path_x;
    evs.previous_path_y = previous_path_y;
};

void EgoVehicle::UpdateTrafficInfo(vector<vector<double> > sensor_fusion) {


}

