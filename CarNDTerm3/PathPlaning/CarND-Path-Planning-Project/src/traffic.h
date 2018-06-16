
#ifndef TRAFFIC_H
#define TRAFFIC_H

#include <vector>
#include "constants.h"


struct TrafficVehicle {

    int    id;
    double pos_x;
    double pos_y;
    double frenet_s;
    double frenet_d;
    double v_x;
    double v_y;
    double speed;

    TrafficVehicle() {
        id       = -1;
        pos_x    = 0.0;
        pos_y    = 0.0;
        frenet_s = 0.0;
        frenet_d = 0.0;
        v_x      = 0.0;
        v_y      = 0.0;
        speed    = 0.0;
    };

    TrafficVehicle(int id, double s, double d, double speed);
    ~TrafficVehicle(){};
};

struct LaneState {
    bool           is_ego_lane;    // Whether this lane is the same lane with the ego vechile
    double         average_speed;  // Average speed of this lane
    int            num_vehicles;
    std::vector<double> best_free_space;
    std::vector<TrafficVehicle> vehicles;

    void Reset() {
      is_ego_lane = false;
      num_vehicles   = 0;
      average_speed   = 0.0;
      best_free_space[0] = 0.0;
      best_free_space[1] = 0.0;
    }

    LaneState() {
        is_ego_lane   = false;
        average_speed = -1.0;
        num_vehicles = 0;
        vehicles = std::vector<TrafficVehicle>(MAX_TRACKED_VEHICLE_A_LANE, TrafficVehicle());
        best_free_space = std::vector<double>(2, 0.0);
    };
    
    ~LaneState(){};
};

class Traffic {
   public:
    Traffic();
    ~Traffic();
    void UpdateTrafficState(const std::vector<std::vector<double>> &sensor_fusion, 
                            int ego_s,
                            int ego_d);

   public:
    std::vector<LaneState> lane_state;
};

#endif  // TRAFFIC_H