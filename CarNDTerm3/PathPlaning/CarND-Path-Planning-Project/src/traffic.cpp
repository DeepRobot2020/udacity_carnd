#include <algorithm>  // std::sort
#include <iostream>

#include "constants.h"
#include "traffic.h"
#include "utils.h"

using namespace std;

#define DEBUG

TrafficVehicle::TrafficVehicle(int id, double s, double d, double speed)
    : id(id), frenet_s(s), frenet_d(d), speed(speed){};

Traffic::Traffic() { lane_state = std::vector<LaneState>(LaneNumberTotal, LaneState()); };

Traffic::~Traffic(){};

void Traffic::UpdateTrafficState(const std::vector<std::vector<double>> &sensor_fusion, 
                                 int ego_s,
                                 int ego_d, 
                                 int remain) {

    for (int lane = 0; lane < lane_state.size(); lane++) { 
        lane_state[lane].Reset(); 
        lane_state[lane].lane = (LaneNumber)lane;
    }

    for (int i = 0; i < sensor_fusion.size(); i++) {
        const int    id       = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_ID];
        double       frenet_s = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_S];
        const float  frenet_d = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_D];
        const double vx       = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_VX];
        const double vy       = sensor_fusion[i][JSON_INDEX_SENSOR_FUSION_VEHICLE_VY];

        const LaneNumber veh_lane = MapFrenetDToLane(frenet_d);
        const LaneNumber ego_lane = MapFrenetDToLane(ego_d);

        if (veh_lane == LaneInvalid) continue;

        const int    lane_slot = lane_state[veh_lane].num_vehicles;
        const double speed     = sqrt(vx * vx + vy * vy);

        frenet_s += ((double)remain * PREDICTION_SUBDURATION * speed);
   
        // Convert to ego-vehicle local frenet coordinate
        const double s_diff = frenet_s - ego_s;

        // if(abs(s_diff) > 3 * MIN_SAFE_DISTRANCE ) continue;
        lane_state[veh_lane].vehicles[lane_slot] = TrafficVehicle(id, s_diff, frenet_d, speed);
        lane_state[veh_lane].num_vehicles++;
        lane_state[veh_lane].average_speed += speed;
        lane_state[veh_lane].ego_vehicle_s = ego_s;
        lane_state[veh_lane].lane = veh_lane;
        lane_state[veh_lane].ego_lane = ego_lane;
    }

    // Process each lane's sensor fusion data to compute the free space & avgerage speed
    for (int lane = 0; lane < LaneNumberTotal; lane++) {

        const int    num_vehicles = lane_state[lane].num_vehicles;
        const double speed_sum    = lane_state[lane].average_speed;
        const double avg_speed    = num_vehicles == 0 ? 22.0 : speed_sum / num_vehicles;

        lane_state[lane].average_speed = avg_speed;

        // No other vechiles in this lane, use very large S to denote very large free space
        double free_space_start = LARGE_S;
        double free_space_end   = -LARGE_S;

        if (num_vehicles == 0) {
            lane_state[lane].best_free_space[0] = free_space_start;
            lane_state[lane].best_free_space[1] = free_space_end;
            continue;
        }

        // Sort based on the relative S value first
        std::sort(lane_state[lane].vehicles.begin(), lane_state[lane].vehicles.end(),
                  [](const TrafficVehicle &a, const TrafficVehicle &b) {
                      return (a.frenet_s < b.frenet_s);
                  });

        // find the first postive frenet_s
        for (int i = 0; i < num_vehicles; i++) {
            TrafficVehicle vehicle = lane_state[lane].vehicles[i];
            // find the first vechile which is ahead of us
            if (vehicle.frenet_s > 0) {
                free_space_start = vehicle.frenet_s;
                // As the array has been sort, so the very previous vehicle should be behind us
                // with negative S, if not then there is no vehicle behind us
                if (i != 0) {
                    TrafficVehicle vehicle2 = lane_state[lane].vehicles[i - 1];
                    if (vehicle2.frenet_s < 0) free_space_end = vehicle2.frenet_s;
                }
                break;
            }
        }

        lane_state[lane].best_free_space[0] = free_space_start;
        lane_state[lane].best_free_space[1] = free_space_end;
    }

    // debug
    for (int i = 0; i < LaneNumberTotal; i++) {

        double free_space_start = lane_state[i].best_free_space[0];
        double free_space_end   = lane_state[i].best_free_space[1];
        // print debug information
        // free space start/end S
        // average speed
        cout << "Lane: " << i << " : "
             << " avg_speed: " << lane_state[i].average_speed 
             << " ego_s: " << ego_s
             << " [ " 
             << free_space_start 
             << ", "
             << free_space_end
             << "]\n";
             

        // for(int j = 0; j < lane_state[i].num_vehicles; j++) {
        //   cout << ", id : " << lane_state[i].vehicles[j].id
        //        << ", s: " << lane_state[i].vehicles[j].frenet_s
        //        << ", speed: " << lane_state[i].vehicles[j].speed;
        // }
    }
}
