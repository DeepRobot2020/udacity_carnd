
#ifndef TRAFFIC_H
#define TRAFFIC_H

#include "constants.h"

struct TrafficVehicle {
    int    id;    double pos_x;
    double pos_y;
    double frenet_s;
    double frenet_d;

    double v_x;
    double v_y;
    double end_path_s;
    double end_path_d;

    TrafficVehicle() {
        frenet_s   = 0.0;
        frenet_d   = 0.0;໮
        pos_x      = 0.0;
        pos_y      = 0.0;
        yaw        = 0.0;
        speed      = 0.0;
        end_path_s = 0.0;
        end_path_d = 0.0;
    };
    ~TrafficVehicle(){};
};

class Traffic {
   public:
    Traffic();
    ~Traffic();

   private:
}

#endif  // TRAFFIC_H